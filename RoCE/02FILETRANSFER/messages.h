#ifndef RDMA_MESSAGES_H
#define RDMA_MESSAGES_H

const char *DEFAULT_PORT = "1234";
const size_t BUFFER_SIZE = 1024 *10;

static char* enumMsgStrings[] = {"MEM_INVALID","MSG_MEMNEXT","MSG_DONE","MSG_READYTOSEND","MSG_READYTORECEIVE"  };
static char* enumWCOPcodeStrings[] = {"IBV_WC_SEND","IBV_WC_RDMA_WRITE","IBV_WC_RDMA_READ","IBV_WC_COMP_SWAP","IBV_WC_FETCH_ADD","IBV_WC_BIND_MW"};

#if defined(DEBUG) 
#define DEBUG_PRINT(fmt, args...) fprintf(stderr, "DEBUG: %s:%d:%s(): " fmt, \
 		     __FILE__, __LINE__, __func__, ##args)
#else
 #define DEBUG_PRINT(fmt, args...) /* Don't do anything in release builds */
#endif

enum message_id
{
  MSG_INVALID = 0,
  MSG_MEMNEXT,
  MSG_DONE,
  MSG_READYTOSEND,
  MSG_READYTORECEIVE,
};

struct message
{
  int id;

  union
  {
    struct
    {
      uint64_t addr;
      uint32_t rkey;
    } mr;
  } data;
  
  char filename[512]; 
};

#endif
