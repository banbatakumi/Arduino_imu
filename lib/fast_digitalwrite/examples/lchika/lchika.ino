// 高速化ライブラリ speedFunctions を用いたLチカ。

#include <speedFunctions2.h>

void setup() {
    pinMode(13, OUTPUT);
}

void loop() {
    high(13); // 13番ピンを HIGH に。
    delay(500);
    low(13); // 13番ピンを LOW に。
    delay(500);
}
