// vnet_backend.c - userspace backend: mmap rings, poll, echo TX->RX
#define _GNU_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <netinet/ether.h>
#include <netinet/ip.h>
#include <arpa/inet.h>

#define VNET_RING_SZ  256
#define VNET_SLOT_SZ  2048

enum vnet_desc_state {
    VNET_DESC_FREE  = 0,
    VNET_DESC_READY = 1,
};

struct vnet_desc {
    uint32_t state;
    uint32_t len;
    uint32_t off;
    uint32_t rsvd;
};

struct vnet_ring {
    uint32_t prod;
    uint32_t cons;
    uint32_t size;
    uint32_t slot_sz;
    struct vnet_desc desc[VNET_RING_SZ];
};

struct vnet_shared {
    struct vnet_ring rx;
    struct vnet_ring tx;
};

#define VNET_IOC_MAGIC 'V'
#define VNET_IOC_KICK_RX _IO(VNET_IOC_MAGIC, 1)

void print_frame(const unsigned char *frame, const char *msg)
{
    struct ether_header *eth = (struct ether_header *)frame;
    char src_mac[18], dst_mac[18];

    ether_ntoa_r((struct ether_addr *)eth->ether_shost, src_mac);
    ether_ntoa_r((struct ether_addr *)eth->ether_dhost, dst_mac);

    if (ntohs(eth->ether_type) == ETHERTYPE_IP) {
        struct iphdr *ip = (struct iphdr *)(frame + sizeof(struct ether_header));
        char src_ip[INET_ADDRSTRLEN], dst_ip[INET_ADDRSTRLEN];

        inet_ntop(AF_INET, &ip->saddr, src_ip, sizeof(src_ip));
        inet_ntop(AF_INET, &ip->daddr, dst_ip, sizeof(dst_ip));

        printf("%s: %s -> %s | %s -> %s | proto %u\n",
               msg, src_mac, dst_mac, src_ip, dst_ip, ip->protocol);
    } else {
        printf("%s: %s -> %s | ethertype 0x%04x\n",
               msg, src_mac, dst_mac, ntohs(eth->ether_type));
    }
}

static int ring_pop(struct vnet_ring *r, uint8_t *dst, uint32_t cap, uint32_t *out_len,
                    uint8_t *data_base)
{
    uint32_t i = r->cons;
    struct vnet_desc *d = &r->desc[i];

    if (d->state != VNET_DESC_READY)
        return -EAGAIN;

    __sync_synchronize(); /* acquire */
    uint32_t len = d->len;
    if (len > cap)
        return -EMSGSIZE;

    memcpy(dst, data_base + d->off, len);

    d->len = 0;
    __sync_synchronize(); /* release */
    d->state = VNET_DESC_FREE;

    r->cons = (i + 1) % r->size;
    *out_len = len;

    return 0;
}

static int ring_push(struct vnet_ring *r, const uint8_t *src, uint32_t len,
                     uint8_t *data_base)
{
   uint32_t i = r->prod;
    struct vnet_desc *d = &r->desc[i];

    if (len > r->slot_sz)
        return -EMSGSIZE;

    if (d->state != VNET_DESC_FREE)
        return -ENOSPC;

    memcpy(data_base + d->off, src, len);
    d->len = len;
    __sync_synchronize(); /* publish */
    d->state = VNET_DESC_READY;

    r->prod = (i + 1) % r->size;
    return 0;
}

int main(int argc, char **argv)
{
    const char *path = "/dev/vnet0";

    int fd = open(path, O_RDWR);
    if (fd < 0) {
        perror("open /dev/vnet0");
        return 1;
    }

    size_t hdr = ((sizeof(struct vnet_shared) + 4095) / 4096) * 4096;
    size_t rx_data_sz = VNET_RING_SZ * VNET_SLOT_SZ;

    size_t need = hdr + rx_data_sz + rx_data_sz;
    size_t shmem_len = (need + 4095) & ~4095;

    void *p = mmap(NULL, shmem_len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    struct vnet_shared *sh = (struct vnet_shared *)p;
    uint8_t *base = (uint8_t *)p;

    uint8_t *rx_data = base + hdr;
    uint8_t *tx_data = base + hdr + rx_data_sz;

    printf("backend running; polling for TX...\n");

    for (;;) {
        struct pollfd pfd = {
            .fd = fd,
            .events = POLLIN,
        };
        int pr = poll(&pfd, 1, 1000);
        if (pr < 0) {
            perror("poll");
            break;
        }

        /* Try to pop TX frames and echo to RX */
        for (;;) {
            uint8_t frame[VNET_SLOT_SZ];
            uint32_t len = 0;

            int ret = ring_pop(&sh->tx, frame, sizeof(frame), &len, tx_data);
            if (ret)
                break;
            printf("backend: got TX frame, len=%u\n", len);
            print_frame(frame, "backend");

            /* Echo back */
            if (ring_push(&sh->rx, frame, len, rx_data) == 0) {
                printf("backend: pushed frame to RX ring\n");
                if (ioctl(fd, VNET_IOC_KICK_RX) < 0)
                    perror("ioctl(KICK_RX)");
                else {
                    printf("backend: kicked RX via ioctl\n");
                }
            }
        }
    }

    munmap(p, shmem_len);
    close(fd);
    return 0;
}
