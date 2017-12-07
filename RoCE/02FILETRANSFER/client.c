#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <rdma/rdma_cma.h>
#include "messages.h"
#include <fcntl.h>


#define TEST_NZ(x) do { if ( (x)) die("error: " #x " failed (returned non-zero)." ); } while (0)
#define TEST_Z(x)  do { if (!(x)) die("error: " #x " failed (returned zero/null)."); } while (0)

const int TIMEOUT_IN_MS = 500; /* ms */
struct context {
  struct ibv_context *ctx;
  struct ibv_pd *pd;
  struct ibv_cq *cq;
  struct ibv_comp_channel *comp_channel;

  pthread_t cq_poller_thread;
};

 struct connection {

  struct ibv_mr *buf_mr;
  
  struct ibv_mr *recv_mr;
  struct ibv_mr *send_mr;
      uint64_t remote_addr;
      uint32_t remote_key;

  char *buf;
  struct message *recv_region;
  struct message *send_region;

};

static void die(const char *reason);

static void build_context(struct ibv_context *verbs);
static void build_qp_attr(struct ibv_qp_init_attr *qp_attr);
static void * poll_cq(void *);
static void post_receives(struct rdma_cm_id *id);
static void register_memory(struct connection *conn);

static int on_addr_resolved(struct rdma_cm_id *id);
static void on_completion(struct ibv_wc *wc);
static int on_connection(struct rdma_cm_id *id);
static int on_disconnect(struct rdma_cm_id *id);
static int on_event(struct rdma_cm_event *event);
static int on_route_resolved(struct rdma_cm_id *id);

static void post_send(struct rdma_cm_id *id);
static void file_transmit(struct rdma_cm_id *id);

static struct context *s_ctx = NULL;
char filename[512];
int fd;
int main(int argc, char **argv)
{
  struct addrinfo *addr;
  struct rdma_cm_event *event = NULL;
  struct rdma_cm_id *conn= NULL;
  struct rdma_event_channel *ec = NULL;
  if (argc != 4)
    die("usage: client <server-address> <server-port> <filename>");
	strncpy(filename,argv[3],strlen(argv[3]));
				fd=open(filename, O_RDONLY);
				if (fd == -1)     
					die("open() failed"); 

  TEST_NZ(getaddrinfo(argv[1], argv[2], NULL, &addr));

  TEST_Z(ec = rdma_create_event_channel());
  TEST_NZ(rdma_create_id(ec, &conn, NULL, RDMA_PS_TCP));
  TEST_NZ(rdma_resolve_addr(conn, NULL, addr->ai_addr, TIMEOUT_IN_MS));

  freeaddrinfo(addr);

  while (rdma_get_cm_event(ec, &event) == 0) {
    struct rdma_cm_event event_copy;

    memcpy(&event_copy, event, sizeof(*event));
    rdma_ack_cm_event(event);

    if (on_event(&event_copy))
      break;
   }

  rdma_destroy_event_channel(ec);

  return 0;
 }

void die(const char *reason)
{
  fprintf(stderr, "%s\n", reason);
  exit(EXIT_FAILURE);
}

void build_context(struct ibv_context *verbs)
{
  if (s_ctx) {
    if (s_ctx->ctx != verbs)
      die("cannot handle events in more than one context.");

    return;
  }

  s_ctx = (struct context *)malloc(sizeof(struct context));

  s_ctx->ctx = verbs;

  TEST_Z(s_ctx->pd = ibv_alloc_pd(s_ctx->ctx));
  TEST_Z(s_ctx->comp_channel = ibv_create_comp_channel(s_ctx->ctx));
  TEST_Z(s_ctx->cq = ibv_create_cq(s_ctx->ctx, 10, NULL, s_ctx->comp_channel, 0)); /* cqe=10 is arbitrary */
  TEST_NZ(ibv_req_notify_cq(s_ctx->cq, 0));

  TEST_NZ(pthread_create(&s_ctx->cq_poller_thread, NULL, poll_cq, NULL));
} 

void build_qp_attr(struct ibv_qp_init_attr *qp_attr)
{
  memset(qp_attr, 0, sizeof(*qp_attr));

  qp_attr->send_cq = s_ctx->cq;
  qp_attr->recv_cq = s_ctx->cq;
  qp_attr->qp_type = IBV_QPT_RC;

  qp_attr->cap.max_send_wr = 10;
  qp_attr->cap.max_recv_wr = 10;
  qp_attr->cap.max_send_sge = 1;
  qp_attr->cap.max_recv_sge = 1;
} 

void * poll_cq(void *ctx)
{ 
  struct ibv_cq *cq;
  struct ibv_wc wc;

  while (1) {
    TEST_NZ(ibv_get_cq_event(s_ctx->comp_channel, &cq, &ctx));
    ibv_ack_cq_events(cq, 1);
    TEST_NZ(ibv_req_notify_cq(cq, 0));

    while (ibv_poll_cq(cq, 1, &wc))
      on_completion(&wc);
  }

  return NULL;
}

void post_send(struct rdma_cm_id *id)
{
	struct connection *conn = (struct connection *)id->context;
	struct ibv_send_wr wr, *bad_wr = NULL;
	struct ibv_sge sge;

	time_t timep;
	time (&timep);

	   printf("connected. posting send...\n");

	   memset(&wr, 0, sizeof(wr));

	   wr.opcode = IBV_WR_SEND;
	   wr.sg_list = &sge;
	   wr.num_sge = 1;
	   wr.send_flags = IBV_SEND_SIGNALED;

	   sge.addr = (uintptr_t)conn->send_region;
	   sge.length =sizeof(struct message);
	   sge.lkey = conn->send_mr->lkey;

	   TEST_NZ(ibv_post_send(id->qp, &wr, &bad_wr));
} 


void post_receives(struct rdma_cm_id *id)
{
  struct connection *conn = (struct connection *)id->context;
  struct ibv_recv_wr wr, *bad_wr = NULL;
  struct ibv_sge sge;

  wr.wr_id = (uintptr_t)id;
  wr.next = NULL;
  wr.sg_list = &sge;
  wr.num_sge = 1;

  sge.addr = (uintptr_t)conn->recv_region;
  sge.length = sizeof (struct message);
  sge.lkey = conn->recv_mr->lkey;

  TEST_NZ(ibv_post_recv(id->qp, &wr, &bad_wr));
} 

void register_memory(struct connection *conn)
{ 
	conn->send_region = malloc(sizeof (struct message));
	conn->recv_region = malloc(sizeof(struct message));
	conn->buf = malloc(BUFFER_SIZE);

	TEST_Z(conn->send_mr = ibv_reg_mr(
				s_ctx->pd, 
				conn->send_region, 
				sizeof(struct message), 
				IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE));

	TEST_Z(conn->recv_mr = ibv_reg_mr(
				s_ctx->pd, 
				conn->recv_region, 
				sizeof(struct message), 
				IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE));


	TEST_Z(conn->buf_mr = ibv_reg_mr(
				s_ctx->pd, 
				conn->buf, 
				BUFFER_SIZE, 
				IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE));

}

int on_addr_resolved(struct rdma_cm_id *id)
{
	struct ibv_qp_init_attr qp_attr;
	struct connection *conn;

	printf("address resolved.\n");

	build_context(id->verbs);
	build_qp_attr(&qp_attr);

	TEST_NZ(rdma_create_qp(id, s_ctx->pd, &qp_attr));

	id->context = conn = (struct connection *)malloc(sizeof(struct connection));

	register_memory(conn);
	post_receives(id);

	TEST_NZ(rdma_resolve_route(id, TIMEOUT_IN_MS));

	return 0;
} 


void on_completion(struct ibv_wc *wc)
{
	struct rdma_cm_id *id = (struct rdma_cm_id *)(uintptr_t)wc->wr_id;
	struct connection *conn;

		DEBUG_PRINT("status code=0X%x  wc->opcode=0X%x %s\n",wc->status,wc->opcode,enumWCOPcodeStrings[wc->opcode]);
	if (wc->status != IBV_WC_SUCCESS)
		die("on_completion: status is not IBV_WC_SUCCESS.");

		DEBUG_PRINT("wc->opcode & IBV_WC_RECV=0X%d \n",wc->opcode& IBV_WC_RECV);
	if (wc->opcode & IBV_WC_RECV){
		conn = (struct connection *)id->context;
//		printf("conn->recv_reigion->id=%d\n",conn->recv_region->id);
		file_transmit(id);
	}
	else
	{
		if (wc->opcode == IBV_WC_SEND){
			printf("send completed successfully.\n");
		}
		if (wc->opcode == IBV_WC_RDMA_WRITE){
		printf("WCRDMAWRITE \n");
//		conn = (struct connection *)id->context;
	//	printf("conn->recv_reigion->id=%d\n",conn->recv_region->id);
	//	printf("conn->send_reigion->id=%d\n",conn->send_region->id);
//		file_transmit(id);
		
		}
	}
}

void file_transmit(struct rdma_cm_id *id){
	struct connection *conn = (struct connection *)id->context;
	DEBUG_PRINT("message: %s receive\n", enumMsgStrings[conn->recv_region->id]);
	char tmp[512];	
	struct timeval tv;
	memset(tmp,'\0',sizeof(tmp));
	gettimeofday (&tv , NULL);
	ssize_t size = 0;
	switch	(conn->recv_region->id){
		case MSG_MEMNEXT: 
			size=0;
			size = read(fd, conn->buf, BUFFER_SIZE);

			if (size == -1){
				on_disconnect(id);
				die("read() failed\n");}
			write_remote(id, size);
			post_receives(id);
			break;

		case MSG_DONE:
			rdma_disconnect(id);
			break;

		case MSG_READYTORECEIVE://sent from server 2, so client need to read the file and change to memnext;

			conn->remote_addr=conn->recv_region->data.mr.addr;
			conn->remote_key=conn->recv_region->data.mr.rkey;
			DEBUG_PRINT(" in file transmit mraddr=%p",conn->recv_region->data.mr.addr);
			DEBUG_PRINT(" in file transmit mrrkey=%p",conn->recv_region->data.mr.rkey);
			post_receives(id);
			break;
	}
}


int write_remote(struct rdma_cm_id *id, uint32_t len){
	struct connection *conn = (struct connection *)id->context;
	struct ibv_send_wr wr, *bad_wr = NULL;
	struct ibv_sge sge;

	DEBUG_PRINT("read %d byte\n", len);

	DEBUG_PRINT("write to remote\n" );
	memset(&wr, 0, sizeof(wr));

	wr.wr_id = (uintptr_t)id;
	wr.opcode = IBV_WR_RDMA_WRITE_WITH_IMM;
	wr.send_flags = IBV_SEND_SIGNALED;
	wr.imm_data = htonl(len);
	wr.wr.rdma.remote_addr = conn->remote_addr;
	wr.wr.rdma.rkey = conn->remote_key;
	DEBUG_PRINT("mraddr=%p ",conn->remote_addr);
	DEBUG_PRINT("mrrkey=%p ",conn->remote_key);

	if (len) {
		wr.sg_list = &sge;
		wr.num_sge = 1;

		sge.addr = (uintptr_t)conn->buf;
		sge.length = len;
		sge.lkey = conn->buf_mr->lkey;
	}
	TEST_NZ(ibv_post_send(id->qp, &wr, &bad_wr));
}

int on_connection(struct rdma_cm_id *id)
{ 
	struct connection *conn = (struct connection *)id->context;
	struct ibv_send_wr wr, *bad_wr = NULL;
	struct ibv_sge sge;

	printf("connected. posting send...\n");
	conn->send_region->id=MSG_READYTOSEND;
	strncpy(conn->send_region->filename,filename,strlen(filename));
	memset(&wr, 0, sizeof(wr));
	wr.wr_id = (uintptr_t)id;
	wr.opcode = IBV_WR_SEND;
	wr.sg_list = &sge;
	wr.num_sge = 1;
	wr.send_flags = IBV_SEND_SIGNALED;

	sge.addr = (uintptr_t)conn->send_region;
	sge.length = sizeof(struct message);
	sge.lkey = conn->send_mr->lkey;

	TEST_NZ(ibv_post_send(id->qp, &wr, &bad_wr));
	printf("connected. posting send...\n");

	return 0;
}

int on_disconnect(struct rdma_cm_id *id)
{ 
	struct connection *conn = (struct connection *)id->context;

	printf("disconnected.\n");

	rdma_destroy_qp(id);

	ibv_dereg_mr(conn->send_mr);
	ibv_dereg_mr(conn->recv_mr);
	ibv_dereg_mr(conn->buf_mr);

	free(conn->send_region);
	free(conn->recv_region);
	free(conn->buf);
	free(conn);

	rdma_destroy_id(id);

	return 1; /* exit event loop */
} 

int on_event(struct rdma_cm_event *event)
{
	int r = 0;

	if (event->event == RDMA_CM_EVENT_ADDR_RESOLVED)
		r = on_addr_resolved(event->id);
	else if (event->event == RDMA_CM_EVENT_ROUTE_RESOLVED)
		r = on_route_resolved(event->id);
	else if (event->event == RDMA_CM_EVENT_ESTABLISHED)
		r = on_connection(event->id);
	else if (event->event == RDMA_CM_EVENT_DISCONNECTED)
		r = on_disconnect(event->id);
	else
		die("on_event: unknown event.");

	return r;
} 

int on_route_resolved(struct rdma_cm_id *id)
{
	struct rdma_conn_param cm_params;

	printf("route resolved.\n");

	memset(&cm_params, 0, sizeof(cm_params));

	cm_params.rnr_retry_count = 7; // infinite try
	//we want the adapter to resend indeﬁnitely if the peer responds
	//with a receiver-not-ready (RNR) error. RNRs happen when a send request is posted before a corresponding
	//receive request is posted on the peer.

	TEST_NZ(rdma_connect(id, &cm_params));

	return 0;
}
