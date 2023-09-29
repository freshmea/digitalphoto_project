# digitalphoto_project
making of digital photo program

- - -
## 2023_9_27
- - -
* 의뢰한 나무 액자 완성
* 전원 연결 및 젯슨 나노 테스트
* 기본 아이디 jetson/jetson
* gthumb 프로그램으로 사진 연속으로 나오게 하기 
* google photo api 연동 
  * 240566316207-lcq43h4vipivrg519crr66pjeqgrbreo.apps.googleusercontent.com
  * credential 파일 생성. google cloud
* 사람이 없을 때는 화면 꺼짐과 최대절전모드 + pir 센서로 화면 켜짐 정상 작동 모드로 진입하기
* 개발 환경 세팅
  * vscode 설치
  * 노트북에서 vscode ssh로 접근 깃허브 푸쉬/풀 작동확인
  * pytorch, cuda, opencv 설치 및 버전 확인 
* 추가 부품 
  * 전원 연결 어댑터 7V 4A 젯슨 나노용
  * 벽에 설치시 필요한 전선, 익스텐션

- - -
## 2023_9_28
- - -
* 64기가 sd 카드의 파티션 변경이 안됨
  * 용량이 차서 다른 작업이 진해 안됨.
  * [참고 포스팅 링크](https://askubuntu.com/questions/345343/gparted-unable-to-satisfy-all-constraints-on-the-partition/345766#345766)
  * 위의 포스팅을 보고 해결함. gparted 프로그램으로 해결
* ros2 iron 을 설치 하기로 함. ubuntu 20.04 버전이라서 잘 안될 수도 있을거 같은데 일단 진행 해 보기로 함.
  * [설치 링크](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
  * ros2 iron humble 설치가 안됨. galactic 으로 설치
  * `sudo apt install ros-galactic-ros-base`
* [opencv 풀스크린코드](https://gist.github.com/ronekko/dc3747211543165108b11073f929b85e)
* 이미지가 바뀔때 마다 어두워지는 효과 추가


- - -
## 2023_9_29
- - -
* google api 에서 자동으로 다운로드 받는 코드 작성. 
  * photo_api_2.py
  * [참고 링크](https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=eziya76&logNo=221340903346)
  * api 에 접속할 때 static_discovery=False 옵션을 주어야 함. 안그러면 에러가 남.
  * 