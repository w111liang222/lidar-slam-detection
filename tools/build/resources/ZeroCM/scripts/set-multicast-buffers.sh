#/bin/bash
#MEM=2097152
MEM=33554432
sudo sysctl -w net.core.rmem_max=$MEM
sudo sysctl -w net.core.rmem_default=$MEM
sudo sysctl -w net.core.wmem_max=$MEM
sudo sysctl -w net.core.wmem_default=$MEM
