#include "contiki.h"
#include "net/rime/rime.h"
#include "net/packetbuf.h"
#include "core/sys/clock.h"

#include <stdio.h>

PROCESS(frame_receiver_process, "RECEIVER");

AUTOSTART_PROCESSES(&frame_receiver_process);

static unsigned long rx_count = 0;

#define BROADCAST

#define RIME_CHANNEL 146
#define RIME_TYPE "broadcast"
static void recv_callback(struct broadcast_conn *c, const linkaddr_t *from);
static struct broadcast_conn bc;
static const struct broadcast_callbacks bc_cb = {recv_callback};

static void recv_callback(struct broadcast_conn *c, const linkaddr_t *from)
{
  // printf("msg got\n");
  // int tx_count = -1;
  // if (packetbuf_datalen() == sizeof(tx_count))
  //   packetbuf_copyto(&tx_count);

  // csw *m;
  // m = packetbuf_dataptr();
  printf("Hey 0x%02x%02x %s\n", from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

  packetbuf_clear();
  packetbuf_set_datalen(sprintf(packetbuf_dataptr(),
                                "%s%x", (char *)"I got ", packetbuf_attr(PACKETBUF_ADDR_SENDER)) +
                        1);

  // clock_delay_usec(1000);
  broadcast_send(&bc);

  // printf("what is %x %x %d %d\n",
  //        packetbuf_attr(PACKETBUF_ADDR_RECEIVER),
  //        packetbuf_attr(PACKETBUF_ADDR_SENDER),
  //        packetbuf_attr(PACKETBUF_ATTR_TIMESTAMP),
  //        packetbuf_attr(PACKETBUF_ATTR_NETWORK_ID));

  // printf("| %s | message received from %02x.%02x (%i) - RSSI = %d\r\n",
  //        (char *)packetbuf_dataptr(),
  //        from->u8[0], from->u8[1], rx_count,
  //        (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));
  // packetbuf_dataptr();
  // packetbuf_clear();
  rx_count++;
}

PROCESS_THREAD(frame_receiver_process, ev, data)
{
  static struct etimer timer;

  PROCESS_EXITHANDLER(broadcast_close(&bc);)

  PROCESS_BEGIN();

  printf("BQU:0:%u:%u\r\n", RTIMER_NOW(), RTIMER_ARCH_SECOND);
  printf("R %d.%d starting...\r\n", linkaddr_node_addr.u8[0],
         linkaddr_node_addr.u8[1]);

  broadcast_open(&bc, RIME_CHANNEL, &bc_cb);

  etimer_set(&timer, CLOCK_SECOND * 5);

  while (1)
  {
    PROCESS_WAIT_EVENT();
    if (ev == PROCESS_EVENT_TIMER)
    {
      // printf("Receiver %lu\r\n", rx_count);
      // print_sys_status(dw_read_reg_64( DW_REG_SYS_STATUS, DW_LEN_SYS_STATUS ));
      etimer_reset(&timer);
    }
  }

  PROCESS_END();
}
