# UMI 데이터 수집

Vive 트래커 + 다이나믹셀 트리거 그리퍼 + USB 손목 카메라를 동시에 기록해서 GR00T N1.7 학습 데이터셋으로 변환. 한손·양손.

## 장비

| 장치 | 연결 |
|---|---|
| Vive Tracker + USB 동글 (손당 1개), 베이스스테이션 (1개 이상) | SteamVR |
| 트리거 XC330-T181-T, 그리퍼 XM430-W350-T (손당 1쌍) | U2D2 (TTL) |
| 손목 카메라 (손당 1개, UVC, MJPG) | USB |
| BT 버튼 (손당 1개) | 블루투스 |

- 다이나믹셀은 한 버스에 데이지체인. ID 1~4 = 왼트리거, 왼그리퍼, 오른트리거, 오른그리퍼. 전원 12V.
- 양손이면 카메라 2대는 PC 본체의 서로 다른 USB 포트에 (한 허브에 2대면 대역폭 부족으로 한 대가 0fps).

## 1. 설치

Ubuntu 20.04 ~ 24.04.
```bash
sudo apt install -y git
git clone https://github.com/JY7/ARAI_UMI.git ~/ARAI_UMI
bash ~/ARAI_UMI/install.sh
```
1. 스크립트가 띄운 Steam 창: 로그인 → SteamVR '설치'
2. PC 로그아웃 → 다시 로그인 (포트·입력 장치 권한 적용 → `id -nG`에 `dialout`, `input` 있어야 함)
3. 스크립트 한 번 더 → 끝에 `SteamVR: OK (no-HMD config applied)`

**아래 `python3` 명령은 모두 새 터미널마다 먼저 `umi` 실행** (이 폴더로 이동 + venv 켜기). ROS2 `setup.bash`를 불러온 터미널에서는 쓰지 않음.

## 2. 장치 연결

**트래커** (처음 1회 페어링)
1. 베이스스테이션: 높은 곳에서 작업 공간을 향하게 (여러 개면 서로 마주 보게)
2. 트래커마다 USB 동글 1개
3. SteamVR 실행: `nohup steam steam://rungameid/250820 >/dev/null 2>&1 &`
4. SteamVR 메뉴(≡) → Devices → Pair Controller → HTC Vive Tracker → 트래커 전원 버튼을 LED가 파랗게 깜빡일 때까지 길게
5. 아이콘이 초록색이면 정상. 다음부터는 전원만 켜면 연결

**BT 버튼** (PC마다 1회, 버튼을 페어링 모드로 켠 뒤)
```bash
bluetoothctl
power on
scan on               # "Ulanzi RC05"의 MAC 확인 → scan off
pair <MAC>
trust <MAC>           # 다시 켜면 자동 재연결
connect <MAC>
exit
```

## 3. 설정 (`config.yaml`)

PC·장비마다 바꿀 값:

| 항목 | 값 찾기 | 키 |
|---|---|---|
| 사용 손 | | `hands` (`[right]` = 한손) |
| 트래커 | `umi && python3 collect.py --list-vive` (SteamVR 실행 중) | `vive.serials` |
| U2D2 포트 | `ls /dev/serial/by-id/` | `dynamixel.port` |
| 카메라 | `watch -n1 ls /dev/v4l/by-path/` 띄워 두고 한 대씩 꽂기 → 새로 생긴 것 중 `-usb-…-video-index0` | `cameras.<손>.device` |
| BT 버튼 | 버튼 연결 후 `umi && python3 collect.py --test-button` (켠 뒤 연결된 버튼은 안 보임) | `button.bluetooth.devices.<손>` (MAC은 따옴표로) |

예 (`config.yaml`에서 이 줄들만 바꿈, 경로는 `ls`에 나온 이름 앞에 폴더까지):
```yaml
hands: [left, right]
vive:
  serials:
    left: LHR-0AB89868
    right: LHR-E0EC229E
dynamixel:
  port: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FAJ1-if00-port0
cameras:
  left:
    device: /dev/v4l/by-path/pci-0000:00:14.0-usb-0:4:1.0-video-index0
  right:
    device: /dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0
button:
  bluetooth:
    devices:
      left: "26:07:02:F5:05:0B"
      right: "26:07:02:F5:1C:45"
```

- 포트·카메라는 `/dev/ttyUSB0`, `/dev/videoN` 대신 위 경로로 (다시 꽂으면 번호가 바뀜). 카메라 경로는 USB 포트 위치라 항상 같은 포트에 꽂기.
- 카메라 화질 (`cameras.<손>`): `width`·`height`·`fps` (1280×720, 30), `exposure` (100µs 단위, 30fps면 333 이하, `null` = 자동: 어두우면 fps 저하), `gamma` (밝기), `wb_temp` (화이트밸런스 K). 노출을 늘리면 모션 블러 → 밝기는 `gamma`로.

## 4. 캘리브레이션 (최초 1회)

| 명령 | 방법 | 다시 할 때 |
|---|---|---|
| `umi && python3 collect.py --calib-dxl` | Enter → 그리퍼가 약하게 열림 끝·닫힘 끝까지 움직임 (손가락 주변 비우기) | 모터 교체 |
| `umi && python3 collect.py --calib-mount` | SteamVR 실행 중. 자세 A: 그리퍼 끝이 바닥을 향하게 세워 들기 → 자세 B: 평소 잡는 자세 (손잡이 아래, 그리퍼 끝은 수평으로 앞). 자세마다 Enter 후 1.5초 정지 | 트래커 재장착 |

값은 `config.yaml`에 저장. 그리퍼 값이 `null`이면 수집이 시작되지 않음.

## 5. 테스트

```bash
umi
python3 collect.py --test-button         # 버튼만: 누를 때마다 줄이 뜨면 정상
python3 collect.py                       # 전체: 버튼 빠르게 2번(또는 o)으로 원점 설정 (6번 그림) 후 Rerun의 그리퍼가 실제와 같이 움직이면 정상
python3 collect.py --no-dxl --no-cam     # 트래커만
python3 collect.py --no-vive --no-cam    # 트리거 → 그리퍼만
python3 collect.py --no-vive --no-dxl    # 카메라만 (좌우 확인)
```
정상: 상태 줄에 tracker·gripper·cam 빨간 경고 없음.

## 6. 수집

```bash
umi
python3 collect.py --data cup         # data/cup에 저장
```

| 입력 | 동작 |
|---|---|
| 버튼 1번 / `space` | 녹화 시작·정지 |
| 버튼 빠르게 2번 / `o` | 원점 설정 |
| 버튼 빠르게 3번 / `d` | 현재(또는 직전) 에피소드 삭제 (되돌릴 수 없음) |
| `t` / `q` | task 변경 / 종료 (mp4 압축이 끝날 때까지 기다림) |

1. 세션 시작 시 원점 설정 (아래). 원점 없이 녹화한 에피소드는 변환에서 빠짐.
2. 에피소드마다: 버튼 → 작업 → 버튼 ("start" / "stop" 음성).
3. "tracking changed, set origin again" 음성 → 원점 다시 (녹화 중이었으면 "discard": 그 에피소드 버림). SteamVR을 켜고 5분쯤 뒤에 원점을 잡으면 덜 생김.
4. 카메라가 빠지면 collect 재실행 (원점 유지).

**원점 = 테이블 모서리 중점, 테이블 면**

![원점 설정](docs/origin.jpg)

- 손잡이 바닥을 책상에 평평하게 세우고, 바닥 뒤 모서리를 테이블 모서리에 맞춤. 손가락은 테이블 안쪽, 모서리와 수직.
- 양손: 두 그리퍼 사이 중점이 원점 → 모서리 중점 기준 좌우 대칭. 한손: 모서리 중점.
- 베이스스테이션을 옮기면 다시 설정.

## 7. 데이터셋 변환

```bash
umi
python3 replay.py cup             # 재생 (Rerun)
python3 check_episode.py cup      # 시간 정렬 → aligned.npz, check.png
python3 to_gr00t.py cup           # → ~/datasets/cup (--out으로 변경)
```
- 안 쓸 에피소드는 폴더 이름 끝에 `_DISCARDED` (빠지는 에피소드와 이유는 `to_gr00t`가 출력).

## 데이터 형식

GR00T LeRobot v2.1, embodiment tag `NEW_EMBODIMENT`, modality config = 데이터셋 폴더의 `umi_modality_config.py`.

| 항목 | 내용 |
|---|---|
| video | 손별 `<hand>_wrist`, 짧은 변 256 (1280×720 → 456×256), 30fps |
| state | 손별 `<hand>_eef` = xyz + rot6d (회전행렬 앞 두 행, 9) / `<hand>_gripper` = 측정 위치 (0 열림 ~ 1 닫힘) |
| action | 같은 키, 16 step (0.53초). eef는 현재 TCP 기준 상대 pose (GR00T가 변환), gripper는 트리거 지령 |
| language | `config.yaml`의 `task` |
| 좌표계 | 원점 (테이블 모서리 중점, x 앞 / y 왼쪽 / z 위) 기준 TCP |
