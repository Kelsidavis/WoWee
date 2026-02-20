FROM ubuntu:24.04

RUN apt-get update && \
	apt install -y \
		cmake \
		build-essential \
		pkg-config \
		git \
		libsdl2-dev \
		libglew-dev \
		libglm-dev \
		libssl-dev \
		zlib1g-dev \
		libavformat-dev \
		libavcodec-dev \
		libswscale-dev \
		libavutil-dev \
		libstorm-dev && \
	rm -rf /var/lib/apt/lists/*

COPY build-wowee.sh /

ENTRYPOINT ./build-wowee.sh
