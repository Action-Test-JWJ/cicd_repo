#!/bin/bash

# VNC 자동 설정 스크립트
# 사용법: chmod +x vnc_setting.sh && ./vnc_setting.sh

set -e  # 에러 발생 시 스크립트 중단

echo "=== VNC 서버 자동 설정을 시작합니다 ==="

# 현재 사용자명 확인
CURRENT_USER=$(whoami)
echo "현재 사용자: $CURRENT_USER"

# 1. 시스템 업데이트 및 XFCE 설치
echo "1. 시스템 업데이트 및 XFCE 설치 중..."
sudo apt update
sudo apt install xfce4 xfce4-goodies -y

# 2. TigerVNC 설치
echo "2. TigerVNC 설치 중..."
sudo apt install tigervnc-standalone-server tigervnc-common tigervnc-tools -y

# 3. GDM3 설정 (Wayland 활성화)
echo "3. GDM3 설정 중..."
sudo cp /etc/gdm3/custom.conf /etc/gdm3/custom.conf.backup 2>/dev/null || true
sudo tee /etc/gdm3/custom.conf > /dev/null << 'EOF'
[daemon]
# Uncomment the line below to force the login screen to use Xorg
WaylandEnable=true
EOF

# 4. VNC 디렉토리 생성
echo "4. VNC 디렉토리 준비 중..."
mkdir -p ~/.vnc

# 5. xstartup 파일 생성
echo "5. VNC xstartup 파일 생성 중..."
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/sh
# --- 환경 정리 ---
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
unset WAYLAND_DISPLAY                       # Wayland 변수 제거 (중요)

# systemd --user 환경에 남은 값들도 제거(있으면)
systemctl --user unset-environment WAYLAND_DISPLAY XDG_SESSION_TYPE DISPLAY 2>/dev/null || true

# --- X11 강제 ---
export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11                      # GTK를 X11로
export QT_QPA_PLATFORM=xcb                  # Qt를 X11로
export DISPLAY="${VNCDISPLAY:-:2}"          # VNC 디스플레이 명시 (tigervnc가 VNCDISPLAY를 줌)

# --- 선택: X 리소스/배경 ---
[ -r "$HOME/.Xresources" ] && xrdb "$HOME/.Xresources"
xsetroot -solid grey

# --- XFCE 시작 (dbus와 함께 한 번에) ---
exec dbus-launch --exit-with-session startxfce4
EOF

# 6. xstartup 실행 권한 설정
echo "6. xstartup 실행 권한 설정 중..."
chmod +x ~/.vnc/xstartup

# 7. VNC 사용자 할당 설정
echo "7. VNC 사용자 할당 설정 중..."
sudo mkdir -p /etc/tigervnc
sudo tee /etc/tigervnc/vncserver.users > /dev/null << EOF
# TigerVNC User assignment
#
# This file assigns users to specific VNC display numbers.
# The syntax is <display>=<username>. E.g.:

:2=$CURRENT_USER
# :2=andrew
# :3=lisa
EOF

# 8. TigerVNC 설정 파일 생성
echo "8. TigerVNC 설정 파일 생성 중..."
cat > ~/.vnc/tigervnc.conf << 'EOF'
$getDefaultFrom = "-display localhost:0";
$localhost = "no";
$SecurityTypes = "VncAuth,TLSVnc"
EOF

# 9. .bashrc에 VNC 별칭 추가
echo "9. .bashrc에 VNC 별칭 추가 중..."
# 기존 별칭이 있는지 확인하고 없으면 추가
if ! grep -q "alias vnc_run=" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# VNC 서버 실행 별칭" >> ~/.bashrc
    echo "alias vnc_run='sudo systemctl start tigervncserver@:2.service'" >> ~/.bashrc
    echo "alias vnc_stop='sudo systemctl stop tigervncserver@:2.service'" >> ~/.bashrc
    echo "alias vnc_status='sudo systemctl status tigervncserver@:2.service'" >> ~/.bashrc
fi

# 10. VNC 비밀번호 설정
echo "10. VNC 비밀번호 설정..."
echo "VNC 접속용 비밀번호를 설정해주세요:"
vncpasswd

# 11. VNC 서버 초기 실행 (테스트)
echo "11. VNC 서버 테스트 실행 중..."
echo "VNC 서버를 테스트로 실행합니다..."
vncserver :2 -localhost no -geometry 1920x1080 -depth 24

echo ""
echo "=== VNC 서버 설정이 완료되었습니다! ==="
echo ""
echo "📋 설정 정보:"
echo "   - VNC 디스플레이: :2"
echo "   - 해상도: 1920x1080"
echo "   - 포트: 5902"
echo "   - 외부 접속: 허용"
echo ""
echo "🚀 사용법:"
echo "   - VNC 서버 시작: vnc_run (또는 vncserver :2 -localhost no -geometry 1920x1080 -depth 24)"
echo "   - VNC 서버 중지: vnc_stop (또는 vncserver -kill :2)"
echo "   - 서비스 상태 확인: vnc_status"
echo ""
echo "🔗 VNC 클라이언트 연결:"
echo "   - 주소: $(hostname -I | awk '{print $1}'):5902"
echo "   - 또는: localhost:5902 (로컬 연결 시)"
echo ""
echo "💡 별칭을 사용하려면 새 터미널을 열거나 'source ~/.bashrc'를 실행하세요."
echo ""
echo "⚠️  참고: 시스템 재부팅 후에는 수동으로 VNC 서버를 시작해야 합니다."

# .bashrc 다시 로드 (현재 세션에서)
source ~/.bashrc 2>/dev/null || true

echo ""
echo "✅ 설정 완료!"