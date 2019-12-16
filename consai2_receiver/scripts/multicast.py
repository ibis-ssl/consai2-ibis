# MIT License
#
# Copyright (c) 2019 SSL-Roots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import  socket

class Multicast():
    def __init__(self, group_addr, port):
        bind_addr = '0.0.0.0'

        # Create a IPv4/UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Avoid error 'Address already in use'.
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Construct a membership_request
        membership_request = socket.inet_aton(group_addr) + socket.inet_aton(bind_addr)

        # Send add membership request to socket
        self.sock.setsockopt(socket.IPPROTO_IP, 
                socket.IP_ADD_MEMBERSHIP, membership_request)

        # Bind the socket to an interfaces
        self.sock.bind((bind_addr, port))

        # Set non-blocking receiving mode
        self.sock.setblocking(False)


    def recv(self, buf_length):
        try:
            buf = self.sock.recv(buf_length)

        except:
            return  False

        return  buf


    def close(self):
        self.sock.close()
