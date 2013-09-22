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


class LinearSampleProcessor(SampleProcessor):
    count = 0;

    bsln = { 'buf': [], 'val': 0, 'siz': 200, 'thresh': 200 };
    avg  = { 'buf': [], 'val': 0, 'siz': 10 };
    mid  = { 'buf': [], 'val': 0, 'siz': 10, 'thresh': 50, 'decay': 0.01 };

    state = 'SNRT';

    ts = { 'action': "", 'val': 0, 'max': 0 };

    def upd_dict(self, d, v):
        d['val'] = self.avg_sample(d['buf'], d['siz'], v);
        return d['val'];


    def scroll_gain(self, val):
        if val < 1500:
            return int(-(5/1500.)*val + 6);
        else:
            return 1


    def generate_action(self):
        # max timestep is a function of the difference

        # when diff is positive, we wish to scroll up
        # when negative, scroll down
        diff = self.mid['val'] - self.avg['val'];

        if abs(diff) < self.mid['thresh']:
            # in noise region -- ignore
            action = "";
            tsmax = 0;
        elif diff > self.mid['thresh']:
            action = "UP";
            tsmax = self.scroll_gain(abs(diff));
        elif diff < self.mid['thresh']:
            action = "DOWN";
            tsmax = self.scroll_gain(abs(diff));


        if tsmax != self.ts['max'] or action != self.ts['action']:
            made_changes = True;
        else:
            made_changes = False;


        if made_changes:
            # fire new action immediately
            self.ts['action'] = action;
            self.ts['max'] = tsmax;
            self.ts['val'] = 1;

            return self.ts['action'];
        elif action != "":
            self.ts['val'] += 1;
            if self.ts['val'] > self.ts['max']:
                self.ts['val'] = 0;
                return self.ts['action'];
            else:
                return "";
        else:
            return "";


    def process_sample(self, v):
        action = "";

        self.upd_dict(self.avg, v);

        if self.state == "SNRT":
            if  self.count < self.bsln['siz']:
                self.upd_dict(self.bsln, v);
                self.count += 1;
            else:
                self.count = 0;
                self.state = "IDLE";

        elif self.state == "IDLE":
            if self.avg['val'] > self.bsln['val'] + self.bsln['thresh']:
                self.count = 0;
                self.mid['val'] = 0;
                self.state = "GETS";
            else:
                # update baseline
                self.state = "IDLE";

        elif self.state == "GETS":
            if self.count < self.mid['siz']:
                self.upd_dict(self.mid, v)
                self.count += 1;
                self.state = "GETS";
            else:
                self.count = 0;
                self.state = "ACTV";
                self.ts['val'] = 0;

        elif self.state == "ACTV":
            # once we have determined where the user's hand is placed,
            # begin processing scroll commands based on upwards or
            # downwards movements

            if v > self.bsln['val'] + self.bsln['thresh']:
                if self.count < self.avg['siz']:
                    # average 10 samples
                    self.count += 1;
                else:
                    action = self.generate_action();

                    self.count = 0;
                    #del self.mid['buf'][:]
                    #self.upd_dict(self.mid, v)
                    #self.mid['val'] = (self.mid['decay'])*v + (1-self.mid['decay'])*self.mid['val'];
                    self.state = "ACTV";
            else:
                # detected reset; go immediately back to idle
                    self.state = "IDLE";

        print self.state, v,  self.avg['val'], self.mid['val'], action


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
    processors = {
            'simple': SampleProcessor,
            'stepwise': StepwiseSampleProcessor,
            'linear': LinearSampleProcessor };


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
