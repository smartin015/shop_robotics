FROM python:3.9

RUN apt-get update && apt-get install -y --no-install-recommends curl libzmq3-dev libczmq-dev gdb && rm -rf /var/lib/apt/lists/* 
RUN useradd -rm -d /home/fw -s /bin/bash -g root -G sudo -u 1000 fw 
USER 1000
RUN python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
ENV PATH="${PATH}:~/.platformio/penv/bin"

