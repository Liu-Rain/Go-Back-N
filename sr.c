#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2  

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications: 
   - removed bidirectional GBN code and other code not used by prac. 
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static int first_seq;               /*record the first seq num of the window*/


/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int index;
  int seqfirst = first_seq;
  int seqlast = (first_seq + WINDOWSIZE-1) % SEQSPACE;
  

  /* if not blocked waiting on ACK */
  if (((seqfirst <= seqlast) && (A_nextseqnum >= seqfirst && A_nextseqnum <= seqlast)) ||
    ((seqfirst > seqlast) && (A_nextseqnum >= seqfirst || A_nextseqnum <= seqlast)))
  {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    if (A_nextseqnum >= seqfirst)
      index = A_nextseqnum - seqfirst;
    else
      index = WINDOWSIZE - seqfirst + A_nextseqnum;
    buffer[index] = sendpkt;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* start timer if first packet in window */
    if (windowcount == 1)
      starttimer(A,RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  int ackcount = 0;
  int i;
  int seqfirst;
  int seqlast;
  int index;
  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* check if new ACK or duplicate */
    seqfirst = first_seq;
    seqlast = (first_seq + WINDOWSIZE - 1) % SEQSPACE;
    int seqfirst = buffer[windowfirst].seqnum;
    int seqlast = buffer[windowlast].seqnum;
    /* check case when seqnum has and hasn't wrapped */
    if (((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
        ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast))) 
    {
      if (packet.acknum >= seqfirst)
            index = packet.acknum - seqfirst;
      else
        index = WINDOWSIZE - seqfirst + packet.acknum;


      if (buffer[index].acknum == NOTINUSE)
      {
        /* packet is a new ACK */
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;
        windowcount--;
        buffer[index].acknum = packet.acknum;
      }
      else
      {
        if (TRACE > 0)
          printf("----A: duplicate ACK received, do nothing!\n");
      }
      if (packet.acknum == seqfirst)
      {
        for (i = 0; i < WINDOWSIZE; i++)
        {
          if (buffer[i].acknum != NOTINUSE && strcmp(buffer[i].payload, "") != 0)
            ackcount++;
          else
            break;
        }

        first_seq = (first_seq + ackcount) % SEQSPACE;

        /*update buffer*/
        for (i = 0; i < WINDOWSIZE; i++)
        {
          if (buffer[i + ackcount].acknum == NOTINUSE || (buffer[i].seqnum + ackcount) % SEQSPACE == A_nextseqnum)
            buffer[i] = buffer[i + ackcount];
        }
        /*Reset timer*/
        stoptimer(A);
        if (windowcount > 0)
          starttimer(A, RTT);
      }
      else
      {
        buffer[index].acknum = packet.acknum;
      }
    }
  }
  else 
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  if (TRACE > 0)
  {
    printf("----A: time out,resend packets!\n");
    printf("---A: resending packet %d\n", (buffer[0]).seqnum);
  }
  tolayer3(A, buffer[0]);
  packets_resent++;
  starttimer(A, RTT);
}       



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;
  first_seq = 0;
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */
static struct pkt B_buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int B_windowfirst, B_windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int B_seqfirst, B_seqlast, B_windowcount;
static int last;
static int B_base; 
/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  int B_seqfirst;
  int B_seqlast;
  int B_index;
  int count = 0;
  int B_base;

  /* if not corrupted and received packet is in order */
  
  if (IsCorrupted(packet)==-1)
  {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    packets_received++;
    /*create sendpkt*/
    /* send an ACK for the received packet */
    sendpkt.acknum = packet.seqnum;
    sendpkt.seqnum = NOTINUSE;
    /* we don't have any data to send.  fill payload with 0's */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';
    /* computer checksum */
    sendpkt.checksum = ComputeChecksum(sendpkt);
    /*send ack*/
    tolayer3(B, sendpkt);
    /* need to check if new packet or duplicate */
    B_seqfirst = B_base;
    B_seqlast = (B_base + WINDOWSIZE-1) % SEQSPACE;

    if (((B_seqfirst <= B_seqlast) && (packet.seqnum >= B_seqfirst && packet.seqnum <= B_seqlast)) ||
        ((B_seqfirst > B_seqlast) && (packet.seqnum >= B_seqfirst || packet.seqnum <= B_seqlast)))
    {

      /*get index*/
      if (packet.seqnum >= B_seqfirst)
        B_index = packet.seqnum - B_seqfirst;
      else
        B_index = WINDOWSIZE - B_seqfirst + packet.seqnum;

      last = last > B_index ? last:B_index;

      /*if not duplicate, save to buffer*/

      if (strcmp(B_buffer[B_index].payload, packet.payload) !=0)
      {
        /*buffer it*/
        packet.acknum = packet.seqnum;
        B_buffer[B_index] = packet;
        /*if it is the base*/
        if (packet.seqnum == B_seqfirst){
          for (i = 0; i < WINDOWSIZE; i++)
          {
            if (B_buffer[i].acknum >= 0 && strcmp(B_buffer[i].payload, "")!= 0)
              count++;
            else
              break;
          }
          /* update state variables */
          B_base = (B_base + count) % SEQSPACE;
          /*update buffer*/
          for (i = 0; i <WINDOWSIZE; i++)
          {
            if ((i + count) <= (last+1))
              B_buffer[i] = B_buffer[i + count];
          }

        }
        /* deliver to receiving application */
        tolayer5(B, packet.payload);
      }
    }
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i;
  expectedseqnum = 0;
  B_nextseqnum = 1;
  B_base = 0;
  B_windowfirst = 0;
  B_windowlast = -1; 
  B_windowcount = 0;
  B_seqfirst = 0;
  B_seqlast = WINDOWSIZE - 1;
  for (i = 0; i < WINDOWSIZE; i++) 
  {
    B_buffer[i].seqnum = NOTINUSE;  /*mark as empty*/ 
  }}
  last = -1;

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}

