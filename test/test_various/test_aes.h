#ifndef __test_aes_h__
#define __test_aes_h__


namespace test_aes {
    void run();
    void test_aes_key();
    void test_aes_encript_with_key0();
    void test_aes_decript_with_key0();
    void test_aes_encript_with_buff0();
    void test_aes_decript_with_buff0();
    void test_aes_mic();
    void test_aes_micverify();
    void test_aes_mic_packet();
}

#endif