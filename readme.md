# Around View Monitor로 Top Down View 영상 제작

21년도 서울대 융합과학기술원 DYROS에서 진행한 동계융합프로그램 인턴 관련업무 AVM_IMG제작 Project에 대한 소스코드입니다.  

## 개발 환경
|OS|사용 언어|사용 IDE|
|:---:|:---:|:---:|
| UBUNTU16.04|C++ |VSCODE |

## 프로젝트 개발 동기

-  


## System Architecture
<p align="center"><img src="https://user-images.githubusercontent.com/56825894/111900443-f3249600-8a75-11eb-86e1-0d31a07ff713.PNG" width="800px"></p>  

<p align="center"><img src="https://user-images.githubusercontent.com/56825894/111900444-f3bd2c80-8a75-11eb-8946-bd358c1b46d3.PNG" width="600px"></p>  

### Project scenario

1. 전후 좌우 이미지를 input으로 넣어준다.
2. intrinsic parameter와 affine parameter를 가지고 ocamcalib_undistort 패캐지를 이용하여 각 이미지의 픽셀들의 camera좌표계 기준 unit vector를 구한다.
3. camera 좌표계 기준 unit vector를 extrinsic parameter를 이용하여 차량 좌표계 기준으로 unit vector를 구한다. 
4. 차량 좌표계 기준으로 각각의 카마라의 위치를 알고 있으므로 카메라 위치를 지나고 3에서 구한 unit vector를 가지는 직선의 방정식을 구한다.
5. 직선의 방정식의 Z값이 0이되는 X, Y좌표의 값을 구한다.
6. AVM로 제작된 이미지에서 보고싶은 영역을(-10M ~ 10M) 정하고 원하는 크기의 이미지(width : 400, height :400)에 투영시킨다.


## 프로젝트 결과

<p align="center"><img src="https://user-images.githubusercontent.com/56825894/111900446-f3bd2c80-8a75-11eb-904a-97938ef5bb12.PNG" width="500px"></p>  
<p align="center"> Fisheye distortion이 있는 이미지를 Top Down View 로 변형한 결과</p>  

<p align="center"><img src="https://user-images.githubusercontent.com/56825894/111957059-e021cc80-8b2e-11eb-8d1c-0f1ac86ad35a.gif" width="500px"></p>  
<p align="center"> backfile로 받은 전후좌우 카메라로 찍은 실내 주차장 이미지를 AVM 이미지로 제작한 결과</p>  

<p align="center"><img src="https://user-images.githubusercontent.com/56825894/111957070-e3b55380-8b2e-11eb-9190-07f66662fcd6.gif" width="500px"></p>  
<p align="center"> backfile로 받은 전후좌우 카메라로 찍은 실외 주차장 이미지를 AVM 이미지로 제작한 결과 </p>  

