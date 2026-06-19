---
marp: true
html: true
theme: default
paginate: true
footer: "컴퓨터과학과코딩 | Dept. of Advanced Defense Technology and Industry | Prof. Daegyu Lee"
style: |
  section {
    justify-content: flex-start;
    padding: 30px 40px 30px 40px;
  }

  footer {
    position:absolute;
    bottom:20px;
    left:0;
    right:0;
    text-align:center;
    font-size:10pt;
    color:#666;
  }

  h1 {
    font-size:26pt;
    margin-bottom:14px;
  }

  h2 {
    color:#4f81bd;
    font-size:18pt;
  }

  p, li {
    font-size:15pt;
    line-height:1.35;
  }

  pre, code {
    font-size:12pt;
  }

  .cols2 {
    display:grid;
    grid-template-columns:1fr 1fr;
    gap:20px;
  }

  table {
    width:100%;
    border-collapse:collapse;
  }

  td, th {
    border:1px solid #cccccc;
    padding:6px;
  }

  th {
    background:#4f81bd;
    color:white;
  }

---

# `Lecture 14: OpenCV Mini Project`
## 컴퓨터는 이미지를 어떻게 볼까?

---

# `1. 오늘의 문제`

오늘은 OpenCV 함수 설명부터 하지 않는다.

먼저 이미지 한 장을 준비한다.

```text
sample_drone.jpg
```

또는 원하는 사진 파일 하나를 사용한다.

오늘 할 일:

<div style="display:flex;align-items:center;gap:8px;margin:18px 0;">
  <div style="background:#4f81bd;color:white;padding:12px 16px;border-radius:6px;font-size:12pt;font-weight:bold;text-align:center;flex:1;">이미지<br>불러오기</div>
  <div style="font-size:22pt;color:#aaa;">→</div>
  <div style="background:#4f81bd;color:white;padding:12px 16px;border-radius:6px;font-size:12pt;font-weight:bold;text-align:center;flex:1;">크기<br>확인</div>
  <div style="font-size:22pt;color:#aaa;">→</div>
  <div style="background:#4f81bd;color:white;padding:12px 16px;border-radius:6px;font-size:12pt;font-weight:bold;text-align:center;flex:1;">픽셀<br>확인</div>
  <div style="font-size:22pt;color:#aaa;">→</div>
  <div style="background:#e67e22;color:white;padding:12px 16px;border-radius:6px;font-size:12pt;font-weight:bold;text-align:center;flex:1;">이미지<br>수정</div>
  <div style="font-size:22pt;color:#aaa;">→</div>
  <div style="background:#27ae60;color:white;padding:12px 16px;border-radius:6px;font-size:12pt;font-weight:bold;text-align:center;flex:1;">결과<br>저장</div>
</div>

질문:

> 사람은 사진을 보고 물체를 인식한다.  
> 컴퓨터는 무엇을 보고 있을까?

---

# `2. 준비`

필요한 라이브러리

```bash
# 실행 위치: Host Ubuntu 또는 개인 Python 환경
pip install opencv-python matplotlib
```

사용할 파일 구조

```text
opencv_project/
│
├── sample_drone.jpg
└── main.py
```

주의:

- 이미지 파일과 `main.py`는 같은 폴더에 둔다.
- 파일 이름이 다르면 코드에서도 이름을 바꿔야 한다.

```python
import cv2
import matplotlib.pyplot as plt
```

---

# `3. 이미지 불러오기`

먼저 이미지를 불러온다.

```python
import cv2

img = cv2.imread("sample_drone.jpg")

print(img)
```

확인할 것:

```text
None 이 출력되면?
```

가능한 원인:

- 파일 이름이 다름
- 파일 위치가 다름
- 확장자가 다름
- 한글 경로 문제

---

# `4. 이미지 크기 확인`

```python
print(img.shape)
```

예상 출력:

```text
(720, 1280, 3)
```

의미:

<div style="display:flex;gap:16px;margin-top:18px;">
  <div style="border:2px solid #4f81bd;border-radius:8px;padding:14px 24px;text-align:center;">
    <div style="font-size:10pt;color:#888;">첫 번째 값</div>
    <div style="font-size:20pt;font-weight:bold;color:#4f81bd;">720</div>
    <div style="font-size:11pt;">height</div>
  </div>
  <div style="border:2px solid #4f81bd;border-radius:8px;padding:14px 24px;text-align:center;">
    <div style="font-size:10pt;color:#888;">두 번째 값</div>
    <div style="font-size:20pt;font-weight:bold;color:#4f81bd;">1280</div>
    <div style="font-size:11pt;">width</div>
  </div>
  <div style="border:2px solid #e67e22;border-radius:8px;padding:14px 24px;text-align:center;">
    <div style="font-size:10pt;color:#888;">세 번째 값</div>
    <div style="font-size:20pt;font-weight:bold;color:#e67e22;">3</div>
    <div style="font-size:11pt;">channel</div>
  </div>
</div>

---

# `5. 문제 1: 전체 픽셀 수 구하기`

이미지의 전체 픽셀 수를 계산해보자.

힌트:

```python
height, width, channel = img.shape
```

작성할 코드:

```python
height, width, channel = img.shape

num_pixels = height * width

print("height:", height)
print("width:", width)
print("channel:", channel)
print("pixels:", num_pixels)
```

생각해보기:

```text
channel은 왜 픽셀 수 계산에 넣지 않았을까?
```

---

# `6. 픽셀 확인하기`

이미지는 숫자의 집합이다.

특정 위치의 픽셀 값을 확인한다.

```python
pixel = img[100, 100]

print(pixel)
```

예상 출력:

```text
[120 80 40]
```

OpenCV의 색상 순서:

```text
B G R
```

일반적으로 우리가 생각하는 RGB와 순서가 다르다.

---

# `7. 문제 2: 특정 위치 색상 바꾸기`

이미지의 일부 영역을 빨간색으로 바꿔보자.

```python
img[100:200, 100:200] = [0, 0, 255]
```

결과 확인:

```python
cv2.imshow("modified", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

질문:

```text
[0, 0, 255]는 왜 빨간색인가?
```

---

# `8. 이미지 출력 방식`

OpenCV 방식:

```python
cv2.imshow("image", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
```

Matplotlib 방식:

```python
plt.imshow(img)
plt.show()
```

주의:

OpenCV는 BGR  
Matplotlib은 RGB 기준

따라서 색상이 이상하게 보일 수 있다.

---

# `9. BGR to RGB`

Matplotlib으로 올바르게 출력하려면 변환한다.

```python
rgb = cv2.cvtColor(
    img,
    cv2.COLOR_BGR2RGB
)

plt.imshow(rgb)
plt.axis("off")
plt.show()
```

정리:

| Library | Color Order |
|---------|-------------|
| OpenCV | BGR |
| Matplotlib | RGB |

---

# `10. 문제 3: 흑백 이미지 만들기`

힌트:

```python
cv2.cvtColor()
```

작성할 코드:

```python
gray = cv2.cvtColor(
    img,
    cv2.COLOR_BGR2GRAY
)

print(gray.shape)

plt.imshow(
    gray,
    cmap="gray"
)

plt.axis("off")
plt.show()
```

확인할 것:

```text
원본 shape: (height, width, 3)
흑백 shape: (height, width)
```

---

# `11. 흑백 이미지는 왜 2차원인가?`

컬러 이미지:

```text
height × width × 3
```

흑백 이미지:

```text
height × width
```

이유:

- 컬러 이미지는 B, G, R 값이 필요하다.
- 흑백 이미지는 밝기 값 하나만 있으면 된다.

예시:

```text
0   → 검정
255 → 흰색
```

---

# `12. 문제 4: 이미지 크기 줄이기`

이미지를 너무 크게 다루면 연산이 느려질 수 있다.

```python
small = cv2.resize(
    img,
    (320, 240)
)

print(small.shape)
```

출력:

```text
(240, 320, 3)
```

주의:

```python
cv2.resize(img, (width, height))
```

순서는 width, height이다.

---

# `13. 문제 5: 관심 영역 자르기`

이미지에서 필요한 부분만 잘라낸다.

```python
crop = img[
    100:400,
    200:600
]
```

의미:

```text
y축: 100부터 399까지
x축: 200부터 599까지
```

출력:

```python
crop_rgb = cv2.cvtColor(
    crop,
    cv2.COLOR_BGR2RGB
)

plt.imshow(crop_rgb)
plt.axis("off")
plt.show()
```

---

# `14. 좌표계 주의`

이미지 좌표는 수학 좌표와 다르다.

```text
왼쪽 위가 (0, 0)
```

<div style="width:420px;height:230px;border:2px solid #4f81bd;position:relative;margin-top:14px;">
  <div style="position:absolute;left:8px;top:6px;font-size:11pt;color:#c0392b;font-weight:bold;">(0, 0)</div>
  <div style="position:absolute;right:12px;top:6px;font-size:11pt;color:#888;">x 증가 →</div>
  <div style="position:absolute;left:8px;bottom:8px;font-size:11pt;color:#888;">↓ y 증가</div>
  <div style="position:absolute;left:130px;top:70px;width:160px;height:80px;border:3px solid #e67e22;border-radius:4px;text-align:center;padding-top:24px;font-size:12pt;color:#e67e22;font-weight:bold;">crop region</div>
</div>

이미지 슬라이싱:

```python
img[y1:y2, x1:x2]
```

---

# `15. 문제 6: 외곽선 찾기`

객체의 경계선을 찾아보자.

1단계: 흑백 변환

```python
gray = cv2.cvtColor(
    img,
    cv2.COLOR_BGR2GRAY
)
```

2단계: Canny Edge

```python
edge = cv2.Canny(
    gray,
    100,
    200
)
```

출력:

```python
plt.imshow(edge, cmap="gray")
plt.axis("off")
plt.show()
```

---

# `16. Canny 값 바꿔보기`

아래 값을 바꿔보자.

```python
edge = cv2.Canny(
    gray,
    50,
    150
)
```

```python
edge = cv2.Canny(
    gray,
    150,
    250
)
```

| Threshold | Result |
|----------|--------|
| 낮음 | 많은 선 검출 |
| 높음 | 강한 선만 검출 |

질문:

```text
어떤 값이 가장 보기 좋은가?
```

---

# `17. 문제 7: 결과 저장하기`

결과 이미지를 파일로 저장한다.

```python
cv2.imwrite(
    "result_gray.jpg",
    gray
)

cv2.imwrite(
    "result_edge.jpg",
    edge
)

cv2.imwrite(
    "result_crop.jpg",
    crop
)
```

확인:

```text
폴더에 이미지 파일이 생성되었는가?
```

---

# `18. 오늘의 최종 미션`

주어진 이미지로 아래 결과를 모두 만들어라.

제출 파일:

```text
result_gray.jpg
result_small.jpg
result_crop.jpg
result_edge.jpg
```

조건:

1. 원본 이미지 크기 출력
2. 전체 픽셀 수 출력
3. 흑백 이미지 생성
4. 320×240 이미지 생성
5. 관심 영역 crop
6. edge detection 수행
7. 결과 이미지 저장

---

# `19. Starter Code`

```python
import cv2
import matplotlib.pyplot as plt

img = cv2.imread("sample_drone.jpg")

if img is None:
    print("Image load failed")
    exit()

height, width, channel = img.shape

print("height:", height)
print("width:", width)
print("channel:", channel)
print("pixels:", height * width)

gray = cv2.cvtColor(
    img,
    cv2.COLOR_BGR2GRAY
)

small = cv2.resize(
    img,
    (320, 240)
)

crop = img[
    100:400,
    200:600
]

edge = cv2.Canny(
    gray,
    100,
    200
)

cv2.imwrite("result_gray.jpg", gray)
cv2.imwrite("result_small.jpg", small)
cv2.imwrite("result_crop.jpg", crop)
cv2.imwrite("result_edge.jpg", edge)
```

---

# `20. 결과 확인용 코드`

```python
images = [
    ("Original", img),
    ("Gray", gray),
    ("Small", small),
    ("Crop", crop),
    ("Edge", edge)
]

for title, image in images:
    plt.figure()
    
    if len(image.shape) == 2:
        plt.imshow(
            image,
            cmap="gray"
        )
    else:
        image_rgb = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2RGB
        )
        plt.imshow(image_rgb)

    plt.title(title)
    plt.axis("off")
    plt.show()
```

---

# `21. 학생별 수정 과제`

아래 중 2개 이상 수정해본다.

1. 다른 이미지 사용
2. crop 영역 변경
3. resize 크기 변경
4. Canny threshold 변경
5. 특정 영역 색상 변경
6. 결과 파일 이름 변경

수정한 내용은 코드 상단에 주석으로 작성한다.

```python
# 내가 수정한 내용
# 1. crop 영역을 드론 중심으로 바꾸었다.
# 2. Canny threshold를 50, 150으로 변경하였다.
```

---

# `22. 오늘 배운 것`

| Concept | Code |
|---------|------|
| 이미지 읽기 | cv2.imread |
| 크기 확인 | img.shape |
| 픽셀 접근 | img[y, x] |
| 색상 변환 | cv2.cvtColor |
| 크기 변경 | cv2.resize |
| 영역 자르기 | img[y1:y2, x1:x2] |
| 외곽선 검출 | cv2.Canny |
| 이미지 저장 | cv2.imwrite |

---

# `23. 마무리`

오늘 확인한 것:

```text
이미지

↓

숫자 배열

↓

픽셀 조작

↓

영상 처리

↓

컴퓨터 비전
```

이미지는 결국 배열이다.
