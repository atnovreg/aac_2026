"""Exercise persisted stream consumption and single-worker lifecycle behavior."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from tools.mpcc_tuner.config import catalog
from tools.mpcc_tuner.jobs import BusyError, JobManager


class FakeProcess:
    """Only replace process launching; requests, logs and events use real files."""
    returncode = None
    pid = 123456789

    def poll(self):
        return self.returncode


class JobTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.manager = JobManager(self.root)
        self.request = catalog()['defaults']
        self.addCleanup(self.close_manager)

    def close_manager(self):
        if self.manager.process is not None:
            self.manager.process.returncode = 0
        self.manager.close()

    def start(self, process=None):
        process = process or FakeProcess()
        with patch('tools.mpcc_tuner.jobs.subprocess.Popen', return_value=process):
            ident = self.manager.start(self.request)
        return ident, process

    def append_event(self, ident, event, newline=True):
        with (self.root/ident/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(event)+('\n' if newline else ''))

    @staticmethod
    def frame(index):
        return {'type': 'frame', 'index': index, 'step': index+1,
                'time_s': (index+1)*.02, 'state': [0.]*7,
                'samples': [[0.]*16], 'prediction': [[0., 0.]], 'progress_m': 0.}

    def test_partial_event_is_retried_without_duplicate_frames(self):
        ident, _ = self.start()
        self.append_event(ident, {'type': 'started', 'directory': '/example/run', 'track_length_m': 100.})
        self.append_event(ident, self.frame(0))
        self.append_event(ident, self.frame(1), newline=False)
        first = self.manager.get(ident)
        self.assertEqual([frame['index'] for frame in first['frames']], [0])
        self.assertEqual(first['run_path'], '/example/run')
        self.assertEqual(first['status'], 'running')
        with (self.root/ident/'events.jsonl').open('a') as stream:
            stream.write('\n')
        second = self.manager.get(ident, after=0)
        self.assertEqual([frame['index'] for frame in second['frames']], [1])
        self.assertEqual(self.manager.get(ident, after=1)['frames'], [])
        self.assertEqual([frame['index'] for frame in self.manager.get(ident)['frames']], [0, 1])

    def test_failed_and_cancelled_summaries_survive_server_restart(self):
        for status in ['failed', 'cancelled']:
            with self.subTest(status=status):
                ident, process = self.start()
                self.append_event(ident, self.frame(0))
                summary = {'completed_lap': False, 'status': status, 'steps': 1,
                           'failure': {'type': 'RuntimeError', 'message': status}}
                self.append_event(ident, {'type': 'finished', 'directory': '/partial/run', 'summary': summary})
                process.returncode = 0
                observed = self.manager.get(ident)
                self.assertEqual(observed['status'], status)
                self.manager.close()
                self.manager = JobManager(self.root)
                reloaded = self.manager.get(ident)
                self.assertEqual(reloaded['status'], status)
                self.assertEqual(reloaded['summary'], summary)
                self.assertEqual(reloaded['error'], status)
                self.assertEqual(len(reloaded['frames']), 1)

    def test_running_request_is_rejected_and_cancel_preserves_immutable_settings(self):
        ident, _ = self.start()
        before = (self.root/ident/'request.json').read_bytes()
        with self.assertRaises(BusyError):
            self.manager.start(self.request)
        self.assertEqual(len(list(self.root.glob('*/job.json'))), 1)
        self.manager.cancel(ident)
        self.assertTrue((self.root/ident/'cancel').is_file())
        self.assertEqual((self.root/ident/'request.json').read_bytes(), before)
        self.assertEqual(self.manager.get(ident)['status'], 'building')

    def test_starting_next_job_finalizes_prior_worker_that_exited_without_event(self):
        ident, process = self.start()
        process.returncode = 17
        next_ident, _ = self.start()
        old = self.manager.get(ident)
        self.assertNotEqual(ident, next_ident)
        self.assertEqual(old['status'], 'failed')
        self.assertIn('17', old['error'])
        self.assertEqual(self.manager.get(next_ident)['status'], 'building')

    def test_second_manager_cannot_mark_live_job_interrupted(self):
        ident, _ = self.start()
        with self.assertRaises(BusyError):
            JobManager(self.root)
        self.assertEqual(self.manager.get(ident)['status'], 'building')

    def test_restart_history_recovers_finished_event_not_polled_before_shutdown(self):
        ident, process = self.start()
        summary = {'completed_lap': True, 'status': 'success', 'steps': 13, 'failure': None}
        self.append_event(ident, {'type': 'finished', 'directory': '/complete/run', 'summary': summary})
        process.returncode = 0
        self.manager.close()
        self.manager = JobManager(self.root)
        history = {job['id']: job for job in self.manager.history()}
        self.assertEqual(history[ident]['status'], 'completed')

    def test_worker_keeps_single_manager_lock_if_server_handle_closes(self):
        real_popen = subprocess.Popen
        def launch_waiting_child(_command, **kwargs):
            # Retain production's descriptor/session setup, replacing only the
            # costly solver with a child that exits when its stdin closes.
            return real_popen([sys.executable, '-c', 'import sys; sys.stdin.read()'],
                              stdin=subprocess.PIPE, **kwargs)
        with patch('tools.mpcc_tuner.jobs.subprocess.Popen', side_effect=launch_waiting_child):
            ident = self.manager.start(self.request)
        process = self.manager.process
        try:
            self.manager.owner_lock.close()  # A crash closes server descriptors.
            with self.assertRaises(BusyError):
                JobManager(self.root)
        finally:
            process.communicate(timeout=5)
        self.manager.close()
        self.manager = JobManager(self.root)
        self.assertEqual(self.manager.get(ident)['status'], 'failed')


if __name__ == '__main__':
    unittest.main()
