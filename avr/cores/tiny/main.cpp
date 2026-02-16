#include <Arduino.h>

__attribute__((section(".init9")))
int main(void)
{
  init();

  setup();

  for (;;)
    loop();

  return 0;
}
