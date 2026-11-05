# AAC 워크스페이스 경로

같은 분류의 [workspace.py](workspace.py)가 상위 `.f1tenth-workspace` 표식을 찾아 데이터·빌드 워크스페이스를 구합니다. Python에서는 `from system.f1tenth.workspace import find_workspace`로 사용합니다.

소스는 AAC의 실제 기능 폴더에서 직접 불러옵니다. 루트에 별도 `tools/` 호환 트리를 만들지 않습니다. [설치 보조](../../operations/f1tenth/setup_workspace.py)는 데이터 폴더와 숨김 Docker 빌드 제외 설정을 준비합니다.
