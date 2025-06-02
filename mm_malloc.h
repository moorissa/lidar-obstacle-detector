# In your project root directory
mkdir -p include
cat > include/mm_malloc.h << 'EOF'
#ifndef _MM_MALLOC_H_INCLUDED
#define _MM_MALLOC_H_INCLUDED

#include <stdlib.h>

static __inline__ void* _mm_malloc(size_t size, size_t alignment) {
    void *ptr;
    if (posix_memalign(&ptr, alignment, size) == 0)
        return ptr;
    else
        return NULL;
}

static __inline__ void _mm_free(void *ptr) {
    free(ptr);
}

#endif
EOF