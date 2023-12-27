ARG IMAGE_VERSION=16.04
FROM ubuntu:$IMAGE_VERSION
USER root

ENV ZCM_HOME /zcm
WORKDIR $ZCM_HOME

RUN apt-get update && apt-get install -y sudo apt-utils

COPY ./scripts/install-deps.sh ./scripts/install-deps.sh

RUN bash -c './scripts/install-deps.sh -s'

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
    update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1

COPY DEBIAN/ ./DEBIAN
COPY LICENSE ./LICENSE
COPY config/ ./config
COPY docs/ ./docs
COPY examples/ ./examples
COPY gen/ ./gen
COPY scripts/ ./scripts
COPY test/ ./test
COPY tools/ ./tools
COPY util/ ./util
COPY waf ./waf
COPY waftools/ ./waftools
COPY wscript ./wscript
COPY zcm/ ./zcm

ENV PATH ${PATH}:/root/.local/bin:$ZCM_HOME/deps/julia/bin
ENV NVM_DIR /root/.nvm

RUN bash -c 'export JAVA_HOME=$(readlink -f /usr/bin/javac | sed "s:/bin/javac::") && \
             [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
             . $ZCM_HOME/deps/cxxtest/.env && \
             ./waf distclean configure --use-all --use-dev && \
             ./waf build && \
             ./waf install && \
             ./waf build_examples'

CMD bash -c 'export JAVA_HOME=$(readlink -f /usr/bin/javac | sed "s:/bin/javac::") && \
             [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" && \
             . $ZCM_HOME/deps/cxxtest/.env && \
             ./test/ci.sh'
