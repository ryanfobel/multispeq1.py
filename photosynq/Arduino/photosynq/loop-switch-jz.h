
// Jon only
// more + commands

{

case hash("on_5v"):
  Serial_Print_Line("turn on 5V for 30 seconds");
  turn_on_5V();
  delay(30000);
  turn_off_5V();
  Serial_Print_Line("done");
  break;

case hash("cut_through"):
  extern int cut_through;
  cut_through = 1;             // used with serial packet mode
  break;

case hash("feed_watchdog"):
  feed_watchdog();
  break;

case hash("start_watchdog"):
  start_watchdog((int)Serial_Input_Long("\r\n+", 1000));     // enter time in minutes
  break;

case hash("stop_watchdog"):
  stop_watchdog();
  break;

case hash("expr"):
  {
    char c[100];
    Serial_Input_Chars(c, "\r\n", 1000, sizeof(c) - 1);  // no plus since that is a operator
    Serial_Printf("%g\n", expr(c));
  }
  break;

case hash("testmode"):
  extern int jz_test_mode;
  jz_test_mode = 1;
  break;

case hash("readonce"):                           // access write once flash
  // bytes in 0xE and 0xF are the serial number
  Serial_Printf("0,E,F = %8.8x %8.8x %8.8x\n", read_once(0x0), read_once(0xe), read_once(0xf));
  //program_once(0xe,0xABCD);
  delay(1);
  //Serial_Printf("0,E,F = %x %x %x\n",read_once(0x0), read_once(0xe), read_once(0xf));
  break;

case hash("set_date"):
  {
    Serial_Print_Line("enter GMT hours+min+sec+day+month+year+");
    int hours, minutes, seconds, days, months, years;
    hours =  Serial_Input_Long("+");
    minutes =  Serial_Input_Long("+");
    seconds =  Serial_Input_Long("+");
    days =  Serial_Input_Long("+");
    months =  Serial_Input_Long("+");
    years =  Serial_Input_Long("+");
    setTime(hours, minutes, seconds, days, months, years);  // set internal time
    Teensy3Clock.set(now());                                // set RTC
    delay(2000);
  }
  // fall through to print
case hash("print_date"):
  // example: 2004-02-12T15:19:21.000Z and also seconds since 1970
  Serial_Printf("{\"device_time\":\"%4.4d-%2.2d-%2.2dT%2.2d:%2.2d:%2.2d.000Z\",\"device_time\":%u}\n", year(), month(), day(), hour(), minute(), second(), now());
  break;

case hash("battery"):
  Serial_Printf("{\"battery\":%d}\n", battery_percent(1));   // test with load
  break;

case hash("scan_i2c"):
  Serial_Print_Line("0x0E = compass, 0x1D = accel, 0x61 = DAC1?, 0x63 = DAC2?, 0x5B = IR, 0x76/77 = BME280s, 0x29 = PAR/color");
  scan_i2c();
  break;

case hash("sleep"):
  Serial_Print_Line("start sleeping");
  delay(500);
  int accel_changed();

  accel_changed();   // get an initial reading
  shutoff();         // turn off most pins and power

  MMA8653FC_standby();                            // sleep accelerometer
  pinMode(18, INPUT);                             // turn off I2C pins
  pinMode(19, INPUT);                             // or use Wire.end()?

  deep_sleep();

#if 0                   // pick mode here
  sleep_mode(60000);    // sleep at lowest level - deep sleep.
#else

  for (;;) {                 // sleep, but poll accelerometer
    // turn I2C back on
    //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_100);  // using alternative wire library
    //delay(100);

    if (accel_changed() /* && !battery_low(0) */)
      break;
    else {
      //pinMode(18, INPUT);       // turn I2C off
      //pinMode(19, INPUT);
      //turn_off_3V3();
      sleep_mode(200);          // sleep for 200 ms (can be much longer if accel interrupts on movement)
    }
  }
#endif

  setup();
  delay(3000);
  Serial_Print_Line("done sleeping");
  delay(500);
  Serial_Flush_Output();
  break;

case hash("pulse"):
  turn_on_5V();                     // is normally off, but many of the below commands need it
  Serial_Print_Line("PULSE4/5 on");
  DAC_set(4, 100);
  DAC_change();
  digitalWriteFast(PULSE4, HIGH);
  DAC_set(5, 100);
  DAC_change();
  digitalWriteFast(PULSE5, HIGH);
  break;

case hash("dac50"):                 // for testing
  turn_on_5V();                     // is normally off, but many of the below commands need it
  Serial_Print_Line("set all DAC outputs to 50%");
  DAC_set(1, 2048);
  DAC_set(2, 2048);
  DAC_set(3, 2048);
  DAC_set(4, 2048);
  DAC_set(5, 2048);
  DAC_set(6, 2048);
  DAC_set(7, 2048);
  DAC_set(8, 2048);
  DAC_set(9, 2048);
  DAC_set(10, 2048);
  DAC_change();
  break;

case hash("adc_check"):                 // for testing
  turn_on_5V();                     // is normally off, but many of the below commands need it
  Serial_Print_Line("All ADC values");
  for (int i = 0; i < 8; ++i) {
    Serial_Printf("AD %d = %d\n", i, AD7689_read(i));
  }
  break;

case hash("compiled"):
  Serial_Printf("Compiled on: %s %s\n", __DATE__, __TIME__);
  break;

case hash("temp"):
  Serial_Printf("BME2801 Temp = %gC, Humidity = %g%%\n", bme1.readTemperature(), bme1.readHumidity());
  Serial_Printf("BME2802 Temp = %gC, Humidity = %g%%\n", bme2.readTemperature(), bme2.readHumidity());
  break;

case hash("memory"):
  {
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) & stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram.
    int FreeRam = stackTop - heapTop;

    Serial_Printf("heap used = %d, heap free = %d, stack + heap free = %d\n", mallinfo().uordblks , mallinfo().fordblks, FreeRam);
  }
  break;

#ifdef PULSE_TEST

case hash("single_pulse"):
  {
    // JZ test - do not remove
    // read and analyze noise on ADC from a single LED pulse
    const int LED = 5;                              // 1 = green, 2 = red, 5 = IR
    const int SAMPLES = 100;
    uint16_t val[SAMPLES];
    Serial_Print_Line("JZ test");
    turn_on_5V();                                   // is normally off, but many of the below commands need it
    DAC_set(LED, 300);                              // set LED intensity
    DAC_change();
    AD7689_set(0);                                  // select ADC channel
    digitalWriteFast(HOLDM, HIGH);                  // discharge cap
    delay(1000);
    noInterrupts();
    digitalWriteFast(LED_to_pin[LED], HIGH);        // turn on LED
    delayMicroseconds(30);                          // allow slow actopulser to stabilize (longer is better)
    digitalWriteFast(HOLDM, LOW);                   // start integrating (could take baseline value here)
    delayMicroseconds(10);                          // measuring width
    digitalWriteFast(LED_to_pin[LED], LOW);         // turn off LED
    delayMicroseconds(60);                          // experimental - some early samples are rising
    uint32_t delta_time = micros();
    AD7689_read_array(val, SAMPLES);                // read values
    delta_time = micros() - delta_time;
    interrupts();
    for (int i = 0; i < SAMPLES; ++i) {
      val[i] += i * 1.4;                                // adjust for droop (about 1 count per sample)
      Serial_Printf(" % d\n", (int)val[i]);
    }
    Serial_Printf("single pulse stdev = % .2f AD counts\n", stdev16(val, SAMPLES));
    Serial_Printf("time = % d usec for % d samples\n", delta_time, SAMPLES);
  }
  break;

case hash("p2p"):
  {
    // JZ test - do not remove
    // read multiple pulses with increasing intensity or pulse width for linearity test
    // with constant DAC value and pulse width, it is good for a pulse-to-pulse stdev test
    const int LED = 5;                              // 1 = green, 2 = red, 3 = yellow, 5 = IR (keep DAC < 100)
    Serial_Print_Line("using delay - wait...");
    turn_on_5V();                                   // is normally off, but many of the below commands need it
    AD7689_set(0);                                  // 0 is main detector
    DAC_set(LED, 700);                              // set initial LED intensity
    DAC_change();
    const int MAX = 100;                            // try a variety of intensities 0 up to 4095
    int count = 0;
    uint16_t data[100];

    for (int i = 0; i < MAX; i += MAX / 100) {
      //DAC_set(LED, i);                              // change LED intensity
      //DAC_change();
      digitalWriteFast(HOLDM, HIGH);                  // discharge cap
      delay(33);                                      // also allows LED to cool and DC filter to adjust
      noInterrupts();
      digitalWriteFast(LED_to_pin[LED], HIGH);        // turn on LED
      delayMicroseconds(10);                          // allow slow actopulser to stabilize
      digitalWriteFast(HOLDM, LOW);                   // start integrating
      delayMicroseconds(i);                           // pulse width (depends on sensitivity needed)
      digitalWriteFast(LED_to_pin[LED], LOW);         // turn off LED
      const int SAMPLES = 21;                         // reduce noise with multiple reads
      uint16_t val[SAMPLES];
      AD7689_read_array(val, SAMPLES);                // read values
      interrupts();
      data[count] = median16(val, SAMPLES);
      if (data[count] >= 65535) break;                // saturated the ADC, no point in continuing
      Serial_Printf(" % d, % d\n", i, data[count]);
      ++count;
    } // for
    // results from each pulse are in data[]
    Serial_Printf("pulse to pulse stdev = % .2f AD counts, first = % d\n\n", stdev16(data, count), data[0]);
  }
  break;

case 4048:
  {
    // JZ test - do not remove
    // read multiple pulses with increasing intensity or pulse width for linearity test
    // with constant DAC value and pulse width, it is good for a pulse-to-pulse stdev test
    const int LED = 5;                              // 1 = green, 2 = red, 3 = yellow, 5 = IR (keep DAC < 100)
    Serial_Print_Line("wait...");
    turn_on_5V();                                   // is normally off, but many of the below commands need it
    AD7689_set(0);                                  // 0 is main detector
    DAC_set(LED, 200);                              // set initial LED intensity
    DAC_change();
    const int MAX = 200;                            // try a variety of intensities 0 up to 4095
    int count = 0;
    uint16_t data[100];

    _meas_light = LED;
    _pulsesize = 30 + 10;                           // account for actopulser delay
    unsigned _pulsedistance = 2000;                 // lower has less jitter

    startTimers(_pulsedistance);        // schedule continuous LED pulses

    for (int i = 1; i < MAX; i += MAX / 100) {
      //DAC_set(LED, i);                              // change LED intensity
      //DAC_change();
      digitalWriteFast(HOLDM, HIGH);                  // discharge cap

      led_off = 0;

      while (led_off == 0) {}                  // wait till pulse is done

      // a pulse completed
      noInterrupts();

      const int SAMPLES = 19;                         // reduce noise with multiple reads
      uint16_t val[SAMPLES];
      AD7689_read_array(val, SAMPLES);                // read values
      interrupts();

      data[count] = median16(val, SAMPLES);
      if (data[count] >= 65535) break;                 // saturated the ADC, no point in continuing
      //Serial_Printf(" % d, % d\n", i, data[count]);
      ++count;
    } // for

    stopTimers();

    // results from each pulse are in data[]
    Serial_Printf("pulse to pulse stdev = % .2f AD counts, first = % d\n\n", stdev16(data, count), data[0]);
  }
  break;
#endif

}
