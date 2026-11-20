"""Compile the actual Wi-Fi session parser and reject replayed/stale requests."""
from pathlib import Path
import subprocess
import tempfile
import unittest

SOURCE = Path(__file__).resolve().parents[1] / 'components/bike_wifi'
HARNESS = r'''
#include <assert.h>
#include <string.h>
#include "wifi_session.h"

int main(void) {
    wifi_session_t session = {0};
    wifi_request_t r;
    const char *hello = "BIKE1 12 0 1 0 hello";
    assert(wifi_request_parse(hello,strlen(hello),&r));
    assert(wifi_session_accept(&session,&r,10,20,1000000,999) == WIFI_HELLO);
    assert(wifi_session_alive(&session,1500000));
    assert(!wifi_session_alive(&session,1500001));
    assert(wifi_session_accept(&session,&r,10,20,1000010,123) == WIFI_REJECT);
    r.sequence=2; r.token=999; r.board_us=1000000; strcpy(r.command,"balance");
    assert(wifi_session_accept(&session,&r,11,20,1000020,123) == WIFI_REJECT);
    assert(wifi_session_accept(&session,&r,10,21,1000020,123) == WIFI_REJECT);
    assert(wifi_session_accept(&session,&r,10,20,1000020,123) == WIFI_COMMAND);
    r.sequence=3; r.board_us=1;
    assert(wifi_session_accept(&session,&r,10,20,1000030,123) == WIFI_REJECT);
    r.board_us=2000000;
    assert(wifi_session_accept(&session,&r,10,20,1000030,123) == WIFI_REJECT);
    r.board_us=1000000; r.token=998;
    assert(wifi_session_accept(&session,&r,10,20,1000030,123) == WIFI_REJECT);
    r.token=999;
    assert(wifi_session_accept(&session,&r,10,20,1600000,123) == WIFI_REJECT);
    r.token=0; strcpy(r.command,"hello");
    assert(wifi_session_accept(&session,&r,10,20,1600000,123) == WIFI_HELLO);
    assert(session.token == 123);  /* Reconnect issues a new token, invalidating old commands. */
    r.sequence=4; r.token=999; strcpy(r.command,"balance"); r.board_us=1600000;
    assert(wifi_session_accept(&session,&r,10,20,1600010,111) == WIFI_REJECT);
    r.token=123; r.board_us=0; strcpy(r.command,"stop");
    assert(wifi_session_accept(&session,&r,10,20,1600010,111) == WIFI_COMMAND);
    const char *bad[] = {"BIKE1 0 0 1 0 hello", "BIKE1 1 0 0 0 hello",
        "BIKE1 1 0 -1 0 stop", "BIKE1 1 0  +1 0 stop", "BIKE1 1 0 1 0 ",
        "BIKE1 1 0 18446744073709551616 0 stop", "BIKE1 1 0 1 0 stop\n", "BAD 1 0 1 0 hello"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i)
        assert(!wifi_request_parse(bad[i],strlen(bad[i]),&r));
    char embedded[]="BIKE1 1 0 1 0 stop\0extra";
    assert(!wifi_request_parse(embedded,sizeof(embedded)-1,&r));
    return 0;
}
'''


class WifiSessionTests(unittest.TestCase):
    def test_handshake_endpoint_sequence_reconnect_age_and_parser(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            harness = folder / 'harness.c'
            harness.write_text(HARNESS)
            executable = folder / 'session'
            subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-I'+str(SOURCE/'include'), str(harness), str(SOURCE/'wifi_session.c'),
                            '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
