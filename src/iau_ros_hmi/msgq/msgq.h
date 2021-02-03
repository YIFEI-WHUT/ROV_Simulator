#ifndef MSGQ_H
#define MSGQ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <nng/nng.h>
// 对底层的 nng 进行封装，便于统一配置使用

typedef struct { nng_socket nsock; }    msgq_pub;
typedef struct { nng_socket nsock; }    msgq_sub;

typedef struct { nng_socket nsock; }    msgq_pull;
typedef struct { nng_socket nsock; }    msgq_push;

typedef struct { nng_socket nsock; }    msgq_rep;
typedef struct { nng_socket nsock; }    msgq_req;


// get error msg by error code
const char* msgq_strerror(int err);

// free memory return from msgq_???_recv function
void msgq_free(unsigned char *buf, size_t sz);

// return 0 OK, otherwise error, using msgq_strerror to get error
// recv is non-blocking except msgq_req_recv, if no data, return 0 and sz = 0

int msgq_pub_open(msgq_pub *pub, const char *url);
int msgq_pub_send(msgq_pub  pub, unsigned char *buf, size_t sz);
int msgq_pub_close(msgq_pub pub);

int msgq_sub_open(msgq_sub *sub, const char *url);
int msgq_sub_recv(msgq_sub  sub, unsigned char **buf, size_t *sz);
int msgq_sub_close(msgq_sub sub);

int msgq_pull_open(msgq_pull *pull, const char *url);
int msgq_pull_recv(msgq_pull  pull, unsigned char **buf, size_t *sz);
int msgq_pull_close(msgq_pull pull);

int msgq_push_open(msgq_push *push, const char *url);
int msgq_push_send(msgq_push  push, unsigned char *buf, size_t sz);
int msgq_push_close(msgq_push push);

int msgq_rep_open(msgq_rep *rep, const char *url);
int msgq_rep_recv(msgq_rep  rep, unsigned char **buf, size_t *sz);
int msgq_rep_send(msgq_rep  rep, unsigned char *buf, size_t sz);
int msgq_rep_close(msgq_rep rep);

int msgq_req_open(msgq_req *req, const char *url);
int msgq_req_send(msgq_req  req, unsigned char *buf, size_t sz);
// blocking recv due to expecting aa response.
int msgq_req_recv(msgq_req  req, unsigned char **buf, size_t *sz);
int msgq_req_close(msgq_req req);

#ifdef __cplusplus
}
#endif

#endif // MSGQ_H
