#!/usr/bin/env python2

import argparse, serial, sys

class SampleProcessor:
    def __init__(self):
        pass;

    def avg_sample(self, buf, w, v):
        buf.append(v);
        if len(buf) > w: buf.pop(0);
        return sum(buf)/float(len(buf));
        
    def process_sample(self, v):
        out = "%d\n" % (voltage);
        sys.stdout.write(out);
        sys.stdout.flush();


class StepwiseSampleProcessor(SampleProcessor):
    count = 0;

    baseline = 0;
    alpha_b = 0.1;

    state = "IDLE";

    statecount = 0;

    avg  = 0;
    avg_b = [];
    avg_w = 25;

    statebase = 0;
    statebase_b = [];
    statebase_w = 10;


    def __init__(self):
        pass;


    def avg_sample(self, buf, w, v):
        buf.append(v);
        if len(buf) > w: buf.pop(0);
        return sum(buf)/float(len(buf));


    def process_sample(self, v):
        action = "";

        print "%s v:%d bl:%d sb:%d avg:%d sc:%d" %(self.state, v,
                self.baseline, self.statebase, self.avg, self.statecount),

        if self.count < 100:
            # training period
            self.baseline += v/100.;
            self.count += 1;
        else:
            self.avg = self.avg_sample(self.avg_b, self.avg_w, v);

            if self.state == "IDLE":
                if self.avg > self.baseline + self.baselinethresh:
                    # get reference point
                    self.statecount = 0;
                    self.statebase = 0;
                    self.state = "WAIT";
                else:
                    # update baseline
                    #self.baseline = (self.alpha_b)*self.avg + (1 - self.alpha_b)*self.baseline;
                    self.state = "IDLE";

            elif self.state == "WAIT":
                if self.statecount < 50:
                    self.statecount += 1;
                else:
                    self.statecount = 0;
                    self.state = "GETS";

            elif self.state == "GETS":
                if self.statecount < self.statebase_w:
                    self.statebase = self.avg_sample(self.statebase_b, self.statebase_w, self.avg);
                    self.statecount += 1;
                    self.state = "GETS";
                else:
                    self.statecount = 0;
                    self.state = "ACTV";

            elif self.state == "ACTV":
                # once we have determined where the user's hand is placed,
                # begin processing scroll commands based on upwards or
                # downwards movements

                if v > self.baseline + self.baselinethresh:
                    if self.statecount < self.avg_w:
                        # average 10 samples
                        self.statecount += 1;
                    else:
                        if self.avg < self.statebase - self.statethresh:
                            action = "UP";
                        elif self.avg > self.statebase + self.statethresh:
                            action = "DOWN";

                        self.statecount = 0;
                        #self.statebase = 0;
                        self.state = "GETS";

                else:
                    # detected reset; go immediately back to idle
                    self.state = "IDLE";

        print "   -> %s v:%d bl:%d sb:%d avg:%d sc:%d" %(
                self.state, v, self.baseline, self.statebase, self.avg,
                self.statecount),
        print "    %s" % action;


if __name__ == "__main__":
    processors = { 'simple': SampleProcessor, 'stepwise': StepwiseSampleProcessor };


    p = argparse.ArgumentParser(description='capture data from an ADC');

    p.add_argument('-p', '--port', dest='port', type=str, default='/dev/ttyACM0',
            action='store', help='serial port');

    p.add_argument('-b', '--baud', dest='baud', type=int, default=230400,
            action='store', help='baud rate');

    p.add_argument('-a', '--algorithm', dest='algorithm', type=str,
            default='stepwise', action='store',
            help="algorithm: %r" %(processors));

    args = p.parse_args();

    sp = processors[args.algorithm]();
    cxn = serial.Serial(args.port, args.baud);

    while cxn.read(1) != chr(0xAA): continue;

    while True:
        i = cxn.read(3);
        c = [ ord(_) for _ in i ];

        voltage = (c[0] << 16) | (c[1] << 8) | (c[2] << 0);
        sp.process_sample(voltage);
        #out = "%d\n" % voltage;
        #sys.stdout.write(out);
        #sys.stdout.flush();

        while cxn.read(1) != chr(0xAA): pass;
