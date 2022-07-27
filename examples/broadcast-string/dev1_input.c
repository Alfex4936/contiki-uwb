#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/serial-line.h"
#include <stdio.h>

#define BROADCAST

#define RIME_CHANNEL 146
#define RIME_TYPE "broadcast"
static struct broadcast_conn bc;

static void recv_callback(struct broadcast_conn *c, const linkaddr_t *from);

static const struct broadcast_callbacks bc_cb = {recv_callback};

// ranging
#include "dw1000.h"
#include "dw1000-ranging.h"
// ranging

// custom
typedef struct
{
  uint8_t src_addr[2];
  uint16_t timestamp;

} csw;
//

#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700
#define UUS_TO_DWT_TIME 65536
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 8

PROCESS(frame_sender_process, "SENDER");

AUTOSTART_PROCESSES(&frame_sender_process);

// #include "sys/ctimer.h"
// struct ctimer my_timer;

static void recv_callback(struct broadcast_conn *c, const linkaddr_t *from)
{
  // ctimer_set(&my_timer, CLOCK_SECOND * 0 + random_rand() % (CLOCK_SECOND * 3), recv_callback, NULL);

  printf("| %s |\r\n",
         (char *)packetbuf_dataptr());
}

typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);

static uint64 get_tx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

PROCESS_THREAD(frame_sender_process, ev, data)
{
  static struct etimer timeout;
  static unsigned long tx_count = 0;
  static int status;

  PROCESS_EXITHANDLER(broadcast_close(&bc);)

  PROCESS_BEGIN();

  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_setpreambledetecttimeout(PRE_TIMEOUT);

  printf("%s sender 0x%02x%02x starting...\r\n",
         RIME_TYPE,
         linkaddr_node_addr.u8[0],
         linkaddr_node_addr.u8[1]);

  broadcast_open(&bc, RIME_CHANNEL, &bc_cb);

  // contiki/tools/sky - make to ignore "SEND n bytes" message

  // etimer_set(&timer, CLOCK_SECOND);

  csw msg;

  while (1)
  {

    // PROCESS_WAIT_EVENT();
    PROCESS_YIELD();
    if (ev == serial_line_event_message)
    {
      tx_count++;
      printf("Sending %s message: %d seq\r\n",
             RIME_TYPE, tx_count);

      // 113 chars

      packetbuf_clear();

      msg.src_addr[0] = linkaddr_node_addr.u8[0];
      msg.src_addr[1] = linkaddr_node_addr.u8[1];
      msg.timestamp = 0;

      packetbuf_copyfrom(&msg, sizeof(csw));
      // packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,
      //                    PACKETBUF_ATTR_PACKET_TYPE_TIMESTAMP);
      packetbuf_set_datalen(sprintf(packetbuf_dataptr(),
                                    "%s", (char *)data) +
                            1);

      broadcast_send(&bc);

      poll_tx_ts = get_tx_timestamp_u64();
      resp_rx_ts = get_rx_timestamp_u64();
      uint32 final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(final_tx_time);
      final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
      printf("%d and %d and %d and %d\n", poll_tx_ts, resp_rx_ts, final_tx_time, final_tx_ts);
    }
  }

  PROCESS_END();
}