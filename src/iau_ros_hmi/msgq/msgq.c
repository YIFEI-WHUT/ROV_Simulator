#include "msgq.h"

#include <stdio.h>

#include <nng/protocol/pipeline0/pull.h>
#include <nng/protocol/pipeline0/push.h>
#include <nng/protocol/pubsub0/pub.h>
#include <nng/protocol/pubsub0/sub.h>
#include <nng/protocol/reqrep0/rep.h>
#include <nng/protocol/reqrep0/req.h>


#define RECONN_MS   1000


const char* msgq_strerror(int err)
{
    return nng_strerror(err);
}

void msgq_free(unsigned char *buf, size_t sz)
{
    return nng_free(buf, sz);
}

static int _msgq_recv(nng_socket nsock, unsigned char **buf, size_t *sz)
{
    int rv = nng_recv(nsock, buf, sz, NNG_FLAG_ALLOC | NNG_FLAG_NONBLOCK);
    if (rv == NNG_EAGAIN) {
        *sz = 0;
        return 0;
    } else {
        return rv;
    }
}

static int _msgq_send(nng_socket nsock, unsigned char *buf, size_t sz)
{
    return nng_send(nsock, buf, sz, 0);
}

int msgq_pub_open(msgq_pub *pub, const char *url)
{
    int rv;
    if ((rv = nng_pub0_open(&pub->nsock)) != 0) {
        return rv;
    }
    if ((rv = nng_listen(pub->nsock, url, NULL, 0)) != 0) {
        nng_close(pub->nsock);
        return rv;
    }
    nng_setopt_int(pub->nsock, NNG_OPT_SENDBUF, 8);
    return 0;
}

int msgq_pub_send(msgq_pub  pub, unsigned char *buf, size_t sz)
{
    return _msgq_send(pub.nsock, buf, sz);
}
int msgq_pub_close(msgq_pub pub)
{
    return nng_close(pub.nsock);
}

int msgq_sub_open(msgq_sub *sub, const char *url)
{
    int rv;
    if ((rv = nng_sub0_open(&sub->nsock)) != 0) {
        return rv;
    }
    if ((rv = nng_dial(sub->nsock, url, NULL, NNG_FLAG_NONBLOCK)) != 0) {
        nng_close(sub->nsock);
        return rv;
    }
    nng_setopt_ms(sub->nsock, NNG_OPT_RECONNMINT, RECONN_MS);
    nng_setopt_ms(sub->nsock, NNG_OPT_RECONNMAXT, RECONN_MS);

    nng_setopt(sub->nsock, NNG_OPT_SUB_SUBSCRIBE, "", 0);
    return 0;
}

int msgq_sub_recv(msgq_sub  sub, unsigned char **buf, size_t *sz)
{
    return _msgq_recv(sub.nsock, buf, sz);
}

int msgq_sub_close(msgq_sub sub)
{
    return nng_close(sub.nsock);
}

int msgq_pull_open(msgq_pull *pull, const char *url)
{
    int rv;
    if ((rv = nng_pull0_open(&pull->nsock)) != 0) {
        return rv;
    }
    if ((rv = nng_listen(pull->nsock, url, NULL, 0)) != 0) {
        nng_close(pull->nsock);
        return rv;
    }
    nng_setopt_int(pull->nsock, NNG_OPT_RECVBUF, 8);
    return 0;
}
int msgq_pull_recv(msgq_pull  pull, unsigned char **buf, size_t *sz)
{
    return _msgq_recv(pull.nsock, buf, sz);
}

int msgq_pull_close(msgq_pull pull)
{
    return nng_close(pull.nsock);
}

int msgq_push_open(msgq_push *push, const char *url)
{
    int rv;
    if ((rv = nng_push0_open(&push->nsock)) != 0) {
        return rv;
    }
    if ((rv = nng_dial(push->nsock, url, NULL, NNG_FLAG_NONBLOCK)) != 0) {
        nng_close(push->nsock);
        return rv;
    }
    nng_setopt_ms(push->nsock, NNG_OPT_RECONNMINT, RECONN_MS);
    nng_setopt_ms(push->nsock, NNG_OPT_RECONNMAXT, RECONN_MS);
    nng_setopt_ms(push->nsock, NNG_OPT_SENDTIMEO, 10);

    return 0;
}
int msgq_push_send(msgq_push  push, unsigned char *buf, size_t sz)
{
    return _msgq_send(push.nsock, buf, sz);
}
int msgq_push_close(msgq_push push)
{
    return nng_close(push.nsock);
}

int msgq_rep_open(msgq_rep *rep, const char *url)
{
    int rv;
    if ((rv = nng_rep0_open(&rep->nsock)) != 0) {
        return rv;
    }
    if ((rv = nng_listen(rep->nsock, url, NULL, 0)) != 0) {
        nng_close(rep->nsock);
        return rv;
    }
    return 0;
}

int msgq_rep_recv(msgq_rep  rep, unsigned char **buf, size_t *sz)
{
    return _msgq_recv(rep.nsock, buf, sz);
}

int msgq_rep_send(msgq_rep  rep, unsigned char *buf, size_t sz)
{
    return _msgq_send(rep.nsock, buf, sz);
}

int msgq_rep_close(msgq_rep rep)
{
    return nng_close(rep.nsock);
}

int msgq_req_open(msgq_req *req, const char *url)
{
    int rv;
    if ((rv = nng_req0_open(&req->nsock)) != 0) {
        return rv;
    }
    if ((rv = nng_dial(req->nsock, url, NULL, NNG_FLAG_NONBLOCK)) != 0) {
        nng_close(req->nsock);
        return rv;
    }
    nng_setopt_ms(req->nsock, NNG_OPT_RECONNMINT, RECONN_MS);
    nng_setopt_ms(req->nsock, NNG_OPT_RECONNMAXT, RECONN_MS);

    nng_setopt_ms(req->nsock, NNG_OPT_SENDTIMEO, 500);
    nng_setopt_ms(req->nsock, NNG_OPT_RECVTIMEO, 1000);
    return 0;
}

int msgq_req_recv(msgq_req  req, unsigned char **buf, size_t *sz)
{
    return nng_recv(req.nsock, buf, sz, NNG_FLAG_ALLOC);
}

int msgq_req_send(msgq_req  req, unsigned char *buf, size_t sz)
{
    return _msgq_send(req.nsock, buf, sz);
}

int msgq_req_close(msgq_req req)
{
    return nng_close(req.nsock);
}
