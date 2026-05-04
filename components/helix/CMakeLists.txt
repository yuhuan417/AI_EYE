idf_component_register(
    SRCS
        libhelix-mp3/bitstream.c libhelix-mp3/buffers.c libhelix-mp3/dct32.c
        libhelix-mp3/dequant.c libhelix-mp3/dqchan.c libhelix-mp3/huffman.c
        libhelix-mp3/hufftabs.c libhelix-mp3/imdct.c libhelix-mp3/mp3dec.c
        libhelix-mp3/mp3tabs.c libhelix-mp3/polyphase.c libhelix-mp3/scalfact.c
        libhelix-mp3/stproc.c libhelix-mp3/subband.c libhelix-mp3/trigtabs.c
    INCLUDE_DIRS . libhelix-mp3
)