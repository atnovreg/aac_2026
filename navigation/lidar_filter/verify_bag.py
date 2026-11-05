"""Reopen source and exported MCAP; check timestamps, metadata and payloads."""
import argparse
import copy
from bisect import bisect_right
import hashlib
import json
import yaml
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from .__main__ import reader_for
from .filtering import records,mapper_effective_timestamps


def verify(output):
    manifest=json.loads((output/'manifest.json').read_text());summary=json.loads((output/'bag_summary.json').read_text())
    src=reader_for(Path(manifest['source_bag']));dst=reader_for(Path(summary['filtered_bag']))
    def metadata(reader):
        return sorted((v.name,v.type,v.serialization_format,v.type_description_hash) for v in reader.get_all_topics_and_types())
    if metadata(src)!=metadata(dst): raise AssertionError('Topic metadata differs')
    def qos_metadata(path):
        data=yaml.safe_load((Path(path)/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        return {item['topic_metadata']['name']:item['topic_metadata'].get('offered_qos_profiles',[]) for item in data['topics_with_message_count']}
    source_qos=qos_metadata(manifest['source_bag']);saved_qos=qos_metadata(summary['filtered_bag'])
    qos_normalizations=[]
    normalized=copy.deepcopy(source_qos)
    for topic,profiles in normalized.items():
        if len(profiles)!=len(saved_qos.get(topic,[])): raise AssertionError('QoS profile count differs')
        for index,profile in enumerate(profiles):
            actual=saved_qos[topic][index]
            # rosbag2_py constructs rclcpp::QoS with default depth10 even for
            # UNKNOWN history + depth0. Depth is unspecified for this history.
            if profile.get('history')=='unknown' and actual.get('history')=='unknown' and profile.get('depth')==0 and actual.get('depth')==10:
                profile['depth']=10
                qos_normalizations.append({'topic':topic,'profile':index,'history':'unknown','source_depth':0,'saved_depth':10})
    if normalized!=saved_qos: raise AssertionError('Recorded offered QoS differs beyond known UNKNOWN-history depth normalization')
    sha_source=hashlib.sha256();sha_saved=hashlib.sha256();lidar_frames=0;other_messages=0;removed_total=0;max_endpoint_change=0
    while src.has_next():
        if not dst.has_next(): raise AssertionError('Output bag ended early')
        ta,da,sa=src.read_next();tb,db,sb=dst.read_next()
        if (ta,sa)!=(tb,sb): raise AssertionError('Topic order or receive timestamp differs')
        if ta!='/livox/lidar':
            if da!=db: raise AssertionError('Non-LiDAR payload changed')
            sha_source.update(ta.encode()+str(sa).encode()+da);sha_saved.update(tb.encode()+str(sb).encode()+db);other_messages+=1
            continue
        a=deserialize_message(da,PointCloud2);b=deserialize_message(db,PointCloud2)
        if a.header!=b.header or a.fields!=b.fields or a.point_step!=b.point_step or a.is_dense!=b.is_dense or a.is_bigendian!=b.is_bigendian: raise AssertionError('PointCloud2 metadata changed')
        before=records(a);after=records(b)
        # Duplicate records are legal (notably no-return points with rounded
        # epoch timestamps). Match the next identical record after the previous
        # source index instead of assuming record/timestamp uniqueness.
        lookup={}
        for i,record in enumerate(before): lookup.setdefault(bytes(record),[]).append(i)
        previous=-1
        for record in after:
            occurrences=lookup.get(bytes(record),[])
            pos=bisect_right(occurrences,previous)
            if pos==len(occurrences):
                raise AssertionError('Output is not an ordered exact subset of source records')
            previous=occurrences[pos]
        before_ts=mapper_effective_timestamps(before);after_ts=mapper_effective_timestamps(after)
        if len(after_ts)<2: raise AssertionError('Too few mapper-usable points')
        max_endpoint_change=max(max_endpoint_change,int(abs(before_ts[-1]-after_ts[-1])))
        lidar_frames+=1;removed_total+=len(before)-len(after)
    if dst.has_next(): raise AssertionError('Extra output messages')
    if removed_total!=summary['removed_points']: raise AssertionError('Removal count mismatch')
    if max_endpoint_change>5_000_000: raise AssertionError('Mapper scan endpoint changed by over5ms')
    result={'status':'passed','lidar_frames':lidar_frames,'non_lidar_messages':other_messages,
            'removed_points':removed_total,'ordered_exact_point_subsets':True,'topic_types_cloud_metadata_and_receive_times_preserved':True,'qos_depth_normalizations':qos_normalizations,
            'non_lidar_source_sha256':sha_source.hexdigest(),'non_lidar_saved_sha256':sha_saved.hexdigest(),
            'max_mapper_endpoint_change_ns':max_endpoint_change}
    (output/'bag_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('output',type=Path);verify(parser.parse_args().output)
