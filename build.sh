sdcc -lstm8 -mstm8 --out-fmt-ihx -c brf24.c && \
sdcc -lstm8 -mstm8 --out-fmt-ihx -c stm8_utility.c && \
sdcc -lstm8 -mstm8 --out-fmt-ihx -c uart.c && \
sdcc -lstm8 -mstm8 --out-fmt-ihx -c spi.c && \
sdcc -lstm8 -mstm8  --out-fmt-ihx main.c spi.rel uart.rel stm8_utility.rel brf24.rel && \
sudo ./stm8flash -cstlinkv2 -pstm8s103f3 -w main.ihx

{
rm *.cdb
rm *.lk
rm *.lst
rm *.map
rm *.rel
rm *.rst
rm *.sym

rm *.asm
rm *.ihx
} &> /dev/null
