#ifndef __HOWLAND_TEST__
#define __HOWLAND_TEST__

/*
 * test0a demos using DAC088S85 broadcast command, raw spi interface (not spi mngr)
 * to control all DAC output to be 1 (max, say 3.3V) and 0 (min, say 0V)
 */
void test0a(void);

/*
 * test1 proves that for a timer compare event, both ppi and interupt handler can take effect.
 */
void test1(void);

/* test2a proves ppi task and interrupt callback can be 
 * simultaneously triggered by the same timer
 * compare event source.
 * test2b demonstrates the using interrupt callback to toggle
 * gpio pin has a roughly 8uS delay compared with ppi task.
 */
void test2a(void);
void test2b(void);

/*
 * test3 sets cycle timer to periodic timer. 2s interval.
 * in each cycle, cycle timer uses ppi to trigger spi timer, which is a one-shot timer.
 * it also trigger count timer increments via ppi.
 * when count timer compares 5, it stops cycle via ppi either.
 */
void test3(void);

/* 
 * test4a toggles channel H.
 * test4b toggles channel G.
 * test4c show channel h and vmid then toogles channel g.
 */
void test4a(void);
void test4b(void);
void test4c(void);

/*
 * test5 is a critical test demostrates 
 * 1. ppi driven ss pin (by timer)
 * 2. ppi driven spi task (by timer)
 * see details in code comment
 */
void test5(void);

/*
 * test6 generates a one-shot waveform containing 9 sequantial spi write.
 * test6a uses 3-channel spi timer and count timer to implement this, with final spi timer count uncleared.
 * test6b fixes the problem by using spi_end instead of spi timer c2 to clear the timer.
 * test6c further clear the count timer (using ORed mask)
 */
void test6a(void);
void test6b(void);
void test6c(void);

/*
 * test7a not fully revamped. it use cycle timer callback to restart test6 multiple spi xfers in each cycle.
 */
void test7a(void);

/*
 * in test8 we are going to construct a full 12-stage spi xfer.
 */
void test8a(void);

/*
 *
 */
void test9a(void);

#endif
