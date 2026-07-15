#!/bin/bash

set -e

# установка средств разработки
[[ -d /var/lib/dpkg || -d /var/lib/apt ]] &&             sudo apt install gcc make autoconf automake libtool git
[[ -d /var/lib/rpm  || -x $(which dnf 2>/dev/null) ]] && sudo dnf install gcc make autoconf automake libtool git

# очистка на случай повторного запуска
./BUILD_CLEANUP.sh || true

# скачивание ...
git clone https://github.com/stephane/libmodbus.git
# ... и сборка зависимостей
SDIR="$(pwd)"
pushd ./libmodbus
  ./autogen.sh
  ./configure --prefix=$SDIR/local_usr
  make && make install
popd

# Сборка проектной работы
make clean
make

# Запуск проекта
[ -x ./solution ] && echo "Попробуйте запустить: ./solution_start.sh"
