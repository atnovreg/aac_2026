# F1TENTH 작업 관리

- [현재 작업 위치와 실행 방법](docs/F1TENTH_시작하기.md)
- [기존 AAC Humble 실행](docs/AAC_HUMBLE.md)
- [장비 설정·AGX 자료 목록](../../system/f1tenth/README_KO.md)
- [원본 작업 보관 위치](archive/README_KO.md)

`setup_workspace.py`는 새 장비의 데이터 폴더·로컬 연결을 준비합니다. `run.sh`는 검사, acados Python, 튜너, 위치 추정 테스트의 실행 입구입니다. `verify_layout.py`는 현재 파일 배치와 데이터 연결만 확인하며 소스 내용을 특정 이관 시점으로 고정하지 않습니다.

```bash
cd ~/ros2_ws/src/aac_2026
bash operations/f1tenth/run.sh check
bash operations/f1tenth/run.sh python -m control.acados_mpcc --help
bash operations/f1tenth/run.sh tuner
bash operations/f1tenth/run.sh localization-test
```

전체 관리 검사:

```bash
bash operations/f1tenth/run.sh python -m unittest discover \
  -s "$PWD/operations/f1tenth/tests" -v
```

일회성 이관 기록·해시 목록·원본 백업·검증 로그는 본체 `~/ros2_ws/log/f1tenth/migration-20261105/`에 보존했습니다. 정리 전 Git 커밋은 `5faa7e0`입니다. 이후 정리의 보존 목록과 검사 결과는 본체 `~/ros2_ws/log/f1tenth/cleanup-20261109/`에 있습니다. 기록을 코드·설정과 함께 복제할 필요는 없습니다.
