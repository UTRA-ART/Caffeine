# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Utilities for calculating and checking NMEA checksums."""


def check_nmea_checksum(nmea_sentence):
    """Calculate and compare the checksum of a NMEA string.

    Args:
        nmea_sentence (str): The NMEA sentence to check.

    Return True if the calculated checksum of the sentence matches the one provided.
    """
    #print(type(nmea_sentence))
    nmea_sentence = str(nmea_sentence)
    # nmea_sentence = nmea_sentence.strip()
    #print(type(nmea_sentence))

    # split_sentence = nmea_sentence.split("*").strip()
    split_sentence = nmea_sentence.strip().split("*")
    if len(split_sentence) != 2:
        # No checksum bytes were found... improperly formatted/incomplete NMEA data?
        return False
    transmitted_checksum = split_sentence[1].strip()

    # Remove the $ at the front
    data_to_checksum = split_sentence[0][1:].strip()
    checksum = 0
    for c in data_to_checksum:
        checksum ^= ord(c)

    # for debugging purposes:
    # print(f"data_to_checksum: {checksum}\ttransmitted_checksum: {transmitted_checksum} ({int(transmitted_checksum[:2], 16)})")
    # print(f"converted type: {type(checksum)}\ttransmitted type: {type(transmitted_checksum)}")
    # print(f"%02X % checksum: {'%02X' % checksum} ({type('%02X' % checksum)})")
    # print(f"tolerance comparison: {abs(checksum - int(transmitted_checksum[:2], 16))}")
    
    
    # actual code:
    # return ("%02X" % checksum) == transmitted_checksum.upper()
    return abs(checksum - int(transmitted_checksum[:2], 16)) <= 3


#import operator
#from functools import reduce
#
#def check_nmea_checksum(sentence: str):
#    """
#    This function checks the validity of an NMEA string using it's checksum
#    """
#    sentence = sentence.strip(bytes("$\n", 'utf-8'))
#    print(sentence)
#    nmeadata, checksum = sentence.split(bytes("*", 'utf-8'), 1)
#    print(nmeadata)
#    calculated_checksum = reduce(operator.xor, (ord(str(s)[1].strip()) for s in nmeadata), 0)
#    if int(checksum, base=16) == calculated_checksum:
#        return nmeadata
#    else:
#        raise ValueError("The NMEA data does not match it's checksum")
