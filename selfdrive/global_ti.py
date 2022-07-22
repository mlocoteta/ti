#!/usr/bin/env python3
from selfdrive.car import gen_empty_fingerprint


global saved_candidate # pylint: disable=W0604
saved_candidate: dict = {} 
global saved_finger # pylint: disable=W0604
saved_finger = gen_empty_fingerprint()
global saved_CarInterface # pylint: disable=W0604
global enabled # pylint: disable=W0604
enabled = False