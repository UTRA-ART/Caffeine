/*!
  \file
  \brief �����O�o�b�t�@

  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_c/urg_ring_buffer.h"


void ring_initialize(ring_buffer_t *ring, char *buffer, const int shift_length)
{
    ring->buffer = buffer;
    ring->buffer_size = 1 << shift_length;
    ring_clear(ring);
}


void ring_clear(ring_buffer_t *ring)
{
    ring->first = 0;
    ring->last = 0;
}


int ring_size(const ring_buffer_t *ring)
{
    int first = ring->first;
    int last = ring->last;

    return (last >= first) ? last - first : ring->buffer_size - (first - last);
}


int ring_capacity(const ring_buffer_t *ring)
{
    return ring->buffer_size - 1;
}


static void byte_move(char *dest, const char *src, int n)
{
    const char *last_p = dest + n;
    while (dest < last_p) {
        *dest++ = *src++;
    }
}


int ring_write(ring_buffer_t *ring, const char *data, int size)
{
    int free_size = ring_capacity(ring) - ring_size(ring);
    int push_size = (size > free_size) ? free_size : size;

    // �f�[�^�z�u
    if (ring->first <= ring->last) {
        // last ���� buffer_size �I�[�܂łɔz�u
        int left_size = 0;
        int to_end = ring->buffer_size - ring->last;
        int move_size = (to_end > push_size) ? push_size : to_end;

        byte_move(&ring->buffer[ring->last], data, move_size);
        ring->last += move_size;
        ring->last &= (ring->buffer_size -1);

        left_size = push_size - move_size;
        if (left_size > 0) {
            // 0 ���� first �̑O�܂ł�z�u
            byte_move(ring->buffer, &data[move_size], left_size);
            ring->last = left_size;
        }
    } else {
        // last ���� first �̑O�܂Ŕz�u
        byte_move(&ring->buffer[ring->last], data, size);
        ring->last += push_size;
    }
    return push_size;
}


int ring_read(ring_buffer_t *ring, char *buffer, int size)
{
    // �f�[�^�擾
    int now_size = ring_size(ring);
    int pop_size = (size > now_size) ? now_size : size;

    if (ring->first <= ring->last) {
        byte_move(buffer, &ring->buffer[ring->first], pop_size);
        ring->first += pop_size;

    } else {
        // first ���� buffer_size �I�[�܂ł�z�u
        int left_size = 0;
        int to_end = ring->buffer_size - ring->first;
        int move_size = (to_end > pop_size) ? pop_size : to_end;
        byte_move(buffer, &ring->buffer[ring->first], move_size);

        ring->first += move_size;
        ring->first &= (ring->buffer_size -1);

        left_size = pop_size - move_size;
        if (left_size > 0) {
            // 0 ���� last �̑O�܂ł�z�u
            byte_move(&buffer[move_size], ring->buffer, left_size);

            ring->first = left_size;
        }
    }
    return pop_size;
}
