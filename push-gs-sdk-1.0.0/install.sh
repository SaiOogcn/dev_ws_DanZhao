#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
INSTALL_DIR="/usr/local"
SERVER="http://47.93.239.40:1215"
CURL_TIMEOUT=30

if [ -n "$SUDO_USER" ]; then
  REAL_USER="$SUDO_USER"
  REAL_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
  REAL_USER="$USER"
  REAL_HOME="$HOME"
fi

CONFIG="$REAL_HOME/.push_gs_config"

if [ -f "$CONFIG" ]; then
  echo "当前机器已安装国赛提交 SDK。"
  echo "如需重新注册，请先执行：push-gs --uninstall"
  exit 1
fi

echo "=============================="
echo "  国赛提交 SDK 安装程序 v1.0.0"
echo "=============================="
echo ""

while true; do
  read -r -p "请输入用户名（仅限英文字母、数字、下划线）: " USERNAME
  if [[ "$USERNAME" =~ ^[a-zA-Z0-9_]+$ ]]; then
    break
  fi
  echo "用户名格式错误，请重新输入。"
done

read -r -p "请输入队伍名称（支持中文和空格）: " TEAM_NAME
if [ -z "$TEAM_NAME" ]; then
  echo "队伍名称不能为空。"
  exit 1
fi

read -r -s -p "请输入密码（至少 6 位）: " PASSWORD
echo ""
read -r -s -p "请再次输入密码: " PASSWORD2
echo ""
if [ "$PASSWORD" != "$PASSWORD2" ]; then
  echo "两次密码输入不一致。"
  exit 1
fi

echo "测试服务器连接..."
if ! curl -s --connect-timeout "$CURL_TIMEOUT" --max-time 60 --retry 3 --retry-connrefused --retry-delay 1 "$SERVER/api/auth/regions/" > /dev/null 2>&1; then
  echo "错误：无法连接到服务器（已自动重试 3 次仍失败）。"
  echo "提示: 若 ping 通但始终连不上，可能是当前网络过滤了端口 1215，请更换网络环境后重试"
  exit 1
fi
echo "服务器连接正常"

echo "正在注册国赛提交账号..."
REGISTER_RESPONSE=$(curl -sS --connect-timeout "$CURL_TIMEOUT" --max-time 120 --retry 2 --retry-connrefused --retry-delay 1 -w "\n%{http_code}" -X POST "$SERVER/api/auth/register/" \
  -H "Content-Type: application/json" \
  -d "{\"username\":\"$USERNAME\",\"password\":\"$PASSWORD\",\"region\":\"GS\",\"team_name\":\"$TEAM_NAME\"}" || true)
REGISTER_STATUS=$(printf '%s' "$REGISTER_RESPONSE" | tail -n 1)
REGISTER_RESULT=$(printf '%s' "$REGISTER_RESPONSE" | sed '$d')

if echo "$REGISTER_RESULT" | grep -q '"token"'; then
  TOKEN=$(echo "$REGISTER_RESULT" | grep -o '"token":"[^"]*"' | cut -d'"' -f4)
  echo "注册成功！"
elif echo "$REGISTER_RESULT" | grep -q 'already exists'; then
  echo "用户已存在，尝试登录..."
  LOGIN_RESPONSE=$(curl -sS --connect-timeout "$CURL_TIMEOUT" --max-time 120 --retry 2 --retry-connrefused --retry-delay 1 -w "\n%{http_code}" -X POST "$SERVER/api/auth/login/" \
    -H "Content-Type: application/json" \
    -d "{\"username\":\"$USERNAME\",\"password\":\"$PASSWORD\",\"region\":\"GS\"}" || true)
  LOGIN_RESULT=$(printf '%s' "$LOGIN_RESPONSE" | sed '$d')
  if echo "$LOGIN_RESULT" | grep -q '"token"'; then
    TOKEN=$(echo "$LOGIN_RESULT" | grep -o '"token":"[^"]*"' | cut -d'"' -f4)
    echo "登录成功！"
  else
    echo "登录失败。"
    exit 1
  fi
else
  echo "注册失败（HTTP $REGISTER_STATUS）"
  [ -n "$REGISTER_RESULT" ] && echo "$REGISTER_RESULT"
  exit 1
fi

{
  printf 'SERVER=%s\n' "$SERVER"
  printf 'USERNAME=%s\n' "$USERNAME"
  printf 'PASSWORD=%s\n' "$PASSWORD"
  printf 'REGION=GS\n'
  printf 'TEAM_NAME=%s\n' "$TEAM_NAME"
  printf 'TOKEN=%s\n' "$TOKEN"
} > "$CONFIG"
chmod 600 "$CONFIG"
REAL_GROUP=$(id -gn "$REAL_USER")
chown "$REAL_USER:$REAL_GROUP" "$CONFIG"

sudo install -d -m 755 "$INSTALL_DIR/lib/push-gs"
sudo install -m 755 "$SCRIPT_DIR/bin/push-core" "$INSTALL_DIR/lib/push-gs/push-core"
sudo install -m 755 "$SCRIPT_DIR/bin/push-gs" "$INSTALL_DIR/bin/push-gs"

echo ""
echo "=============================="
echo "  国赛提交 SDK 安装成功！"
echo "  用户: $USERNAME"
echo "  队伍: $TEAM_NAME"
echo "  服务器目录: GS/GS_${USERNAME}_GS_${TEAM_NAME}"
echo "=============================="
echo ""
echo "使用方法："
echo "  push-gs /path/to/file"
echo "  push-gs /path/to/folder/"
echo "  push-gs --config"
