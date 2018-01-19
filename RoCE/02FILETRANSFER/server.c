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
    int fd;
    char *buf;  //sice it is a file transfer program, we need a buffer
    struct message *recv_region;	//to store the received message
    struct message *send_region;  //to send the message
};

static void die(const char *reason);

static void build_context(struct ibv_context *verbs);
static void build_qp_attr(struct ibv_qp_init_attr *qp_attr);
static void * poll_cq(void *);
static void post_receives(struct rdma_cm_id *id);
static void register_memory(struct connection *conn);

static void file_transmit(struct rdma_cm_id *id);
static void on_completion(struct ibv_wc *wc);
static int on_connect_request(struct rdma_cm_id *id);
static int on_connection(struct rdma_cm_id *id);
static int on_disconnect(struct rdma_cm_id *id);
static int on_event(struct rdma_cm_event *event);
static void post_receives_ack(struct rdma_cm_id *id);
static void post_send(struct rdma_cm_id *id);

static struct context *s_ctx = NULL;

int main(int argc, char **argv)
{

    clock_t tic = clock();

#if _USE_IPV6
    struct sockaddr_in6 addr;
#else
    struct sockaddr_in addr;
#endif
    struct rdma_cm_event *event = NULL;
    struct rdma_cm_id *listener = NULL;
    struct rdma_event_channel *ec = NULL;
    uint16_t port = 0;

    memset(&addr, 0, sizeof(addr));
#if _USE_IPV6
    addr.sin6_family = AF_INET6;
#else
    addr.sin_family = AF_INET;
#endif

    TEST_Z(ec = rdma_create_event_channel());
    TEST_NZ(rdma_create_id(ec, &listener, NULL, RDMA_PS_TCP));
    TEST_NZ(rdma_bind_addr(listener, (struct sockaddr *)&addr));
    TEST_NZ(rdma_listen(listener, 10)); /* backlog=10 is arbitrary */

    port = ntohs(rdma_get_src_port(listener));

    printf("listening on port %d.\n", port);

    while (rdma_get_cm_event(ec, &event) == 0) {
        struct rdma_cm_event event_copy;

        memcpy(&event_copy, event, sizeof(*event));
        rdma_ack_cm_event(event);

        if (on_event(&event_copy))
            break;
    }

    rdma_destroy_id(listener);
    rdma_destroy_event_channel(ec);

    clock_t toc = clock();
    printf("Elapsed: %f seconds\n", (double)(toc - tic) / CLOCKS_PER_SEC);

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

    memset(&wr, 0, sizeof(wr));

    wr.opcode = IBV_WR_SEND;
    wr.sg_list = &sge;
    wr.num_sge = 1;
    wr.send_flags = IBV_SEND_SIGNALED;

    sge.addr = (uintptr_t)conn->send_region;
    sge.length = sizeof (struct message);
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

void post_receives_ack(struct rdma_cm_id *id)
{
    struct ibv_recv_wr wr, *bad_wr = NULL;

    wr.wr_id = (uintptr_t)id;
    wr.next = NULL;
    wr.sg_list = NULL;
    wr.num_sge = 0;

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



/*on_cimpletion
 *1.we check the status code to identify what we receives
 *2.if it is a IBV_WC_RECV_RDMA_WITH_IMM, that means write
 *	further, the size tells how much bytes need to be written
 *3.if it is not a write operation, that means it is the control message (MSG_DONE, MSG_READYTOSEND), so we call file_transmit
*/

void on_completion(struct ibv_wc *wc)
{
    struct rdma_cm_id *id = (struct rdma_cm_id *)(uintptr_t)wc->wr_id;
    struct connection *conn;

    DEBUG_PRINT("status code=0X%x  wc->opcode=0X%x %s \n",wc->status,wc->opcode,enumWCOPcodeStrings[wc->opcode]);

    if (wc->status != IBV_WC_SUCCESS)
        die("on_completion: status is not IBV_WC_SUCCESS.");

    if (wc->opcode & IBV_WC_RECV) {
        conn = (struct connection *)id->context;
        if (wc->opcode == IBV_WC_RECV_RDMA_WITH_IMM) {
            conn = (struct connection *)id->context;
            uint32_t size = ntohl(wc->imm_data);
            DEBUG_PRINT("write %d size to file\n",size);
            if(size!=0) {
                write(conn->fd, conn->buf, size);
                post_receives_ack(id);
                conn->send_region->id=MSG_MEMNEXT;
                post_send(id);
            }
            else {
                conn->send_region->id=MSG_DONE;
                post_send(id);
            }
        }
        else {
            file_transmit(id);
        }
    }
    if (wc->opcode == IBV_WC_SEND) {
        printf("send completed successfully.\n");
    }
}

void file_transmit(struct rdma_cm_id *id) {
    struct connection *conn = (struct connection *)id->context;

    DEBUG_PRINT("message: %s receive\n", enumMsgStrings[conn->recv_region->id]);
    char tmp[512];
    char tmpp[512];
    struct timeval tv;
    memset(tmp,'\0',sizeof(tmp));
    gettimeofday (&tv , NULL);

    switch(conn->recv_region->id) {
    case MSG_DONE:
        close(conn->fd);
        rdma_disconnect(id);
        break;

    case MSG_READYTOSEND:// sent from client
        strncpy(tmp,conn->recv_region->filename,sizeof(conn->recv_region->filename));
        DEBUG_PRINT("recved file_name=%s\n",tmp);
        //fd=open("test123", O_WRONLY | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        conn->fd=open(tmp,O_CREAT | O_RDWR, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        //fd=open(conn->recv_region->filename, O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        if (conn->fd == -1)
            fprintf(stderr,"open() failed\n");
        else
        {
            //conn->send_region->id=MSG_READYTORECEIVE;
            conn->send_region->id=MSG_MEMNEXT;
            //notify remote can start transmitting
        }
        post_receives(id);
        post_send(id);
        break;
    }
    DEBUG_PRINT("message: %s  %d sent\n", enumMsgStrings[conn->send_region->id],conn->send_region->id);
}


int on_connect_request(struct rdma_cm_id *id)
{
    struct ibv_qp_init_attr qp_attr;
    struct rdma_conn_param cm_params;
    struct connection *conn;

    printf("received connection request.\n");

    build_context(id->verbs);
    build_qp_attr(&qp_attr);

    TEST_NZ(rdma_create_qp(id, s_ctx->pd, &qp_attr));

    id->context = conn = (struct connection *)malloc(sizeof(struct connection));

    register_memory(conn);
    post_receives(id);

    memset(&cm_params, 0, sizeof(cm_params));
    cm_params.rnr_retry_count = 7;
    TEST_NZ(rdma_accept(id, &cm_params));

    return 0;
}

int on_connection(struct rdma_cm_id *id)
{
    //struct connection *conn = (struct connection *)context;
    struct connection *conn = (struct connection *)id->context;
    struct ibv_send_wr wr, *bad_wr = NULL;
    struct ibv_sge sge;


    printf("connected. posting send...\n");

    memset(&wr, 0, sizeof(wr));

    wr.wr_id = (uintptr_t)id;
    wr.opcode = IBV_WR_SEND;
    wr.sg_list = &sge;
    wr.num_sge = 1;
    wr.send_flags = IBV_SEND_SIGNALED;

    sge.addr = (uintptr_t)conn->send_region;
    sge.length = sizeof (struct message);
    sge.lkey = conn->send_mr->lkey;
    conn->send_region->id=MSG_READYTORECEIVE;
    conn->send_region->data.mr.rkey=conn->buf_mr->rkey;
    conn->send_region->data.mr.addr=(uintptr_t)conn->buf_mr->addr;
    DEBUG_PRINT("mraddr=%p",conn->send_region->data.mr.addr);
    DEBUG_PRINT("mrrkey=%p",conn->send_region->data.mr.rkey);

    TEST_NZ(ibv_post_send(id->qp, &wr, &bad_wr));
    printf("connected. posting send...\n");

    return 0;
}

int on_disconnect(struct rdma_cm_id *id)
{
    struct connection *conn = (struct connection *)id->context;

    printf("peer disconnected.\n");

    rdma_destroy_qp(id);

    ibv_dereg_mr(conn->send_mr);
    ibv_dereg_mr(conn->recv_mr);
    ibv_dereg_mr(conn->buf_mr);

    free(conn->send_region);
    free(conn->recv_region);
    free(conn->buf);

    free(conn);

    rdma_destroy_id(id);

    return 0;
}

int on_event(struct rdma_cm_event *event)
{
    int r = 0;

    if (event->event == RDMA_CM_EVENT_CONNECT_REQUEST)

        r = on_connect_request(event->id);
    else if (event->event == RDMA_CM_EVENT_ESTABLISHED)
        r = on_connection(event->id);
    else if (event->event == RDMA_CM_EVENT_DISCONNECTED)
        r = on_disconnect(event->id);
    else
        die("on_event: unknown event.");

    return r;
}

