"""
Simple and lightweight statistics collection module.
Logging is done in a separate process to minimize influence of logging on program execution.
"""

from multiprocessing import Process, Queue
from queue import Empty
import csv
from enum import Enum
import datetime
import logging
import time
import sys
import os

# target sleep time in between counter measurements in ms
TARGET_SLEEP_TIME = 1000

class Stats:
    """
    Class for statistics collection.
    """

    class MessageType(Enum):
        """
        Message types used for multiprocess communication.
        """

        STOP = 0
        NEW_LOGGER = 1
        NEW_COUNTER = 2
        NEW_AVERAGER = 3
        NEW_LOGGER_VAL = 4
        NEW_COUNTER_VAL = 5
        NEW_AVERAGER_VAL = 6
        SUBFOLDER = 7
        STARTTIME = 8

    class Message():
        """
        Messages sent to logger thread of stats.
        """
        def __init__(self, typ, name='', value=0, timestamp=0):
            self.type = typ
            self.name = name
            self.value = value
            self.timestamp = timestamp

        def get_type(self):
            """
            Type of the logger.
            """
            return self.type

        def get_name(self):
            """
            Name of the logger
            """
            return self.name

        def get_value(self):
            """
            Value of the logger
            """
            return self.value

        def get_timestamp(self):
            """
            Timestamp of the measured value.
            """
            return self.timestamp

    class StatisticsThread():
        """
        Prints and writes statistics collected asynchrously.
        """
        def __init__(self, queue, stats, subfolder):
            self.queue = queue
            self.stats = stats
            self.loggers = {}
            self.counters = {}
            self.averagers = {}
            self.subfolder = subfolder
            self.starttime = ''
            self.process = Process(target=self.run, args=(self.queue, self.stats, self.subfolder))
            self.process.start()

        def print_averagers(self):
            """
            Write the value of every averager registered to a .csv file.
            """
            path = self.subfolder + '/' + self.starttime + '_averagers.csv'
            if not os.path.exists(path):
                return
            if list(self.averagers.keys()):
                lst = [int(time.monotonic()*1000)]
                for avg in list(self.averagers.keys()):
                    lst.append(self.averagers[avg])
                with open(path, 'a+', newline='') as csvfile:
                    csvwriter = csv.writer(csvfile, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    csvwriter.writerow(lst)

        def print_counters(self):
            """
            Write the value of every counter registered to a .csv file.
            """
            path = self.subfolder + '/' + self.starttime + '_counters.csv'
            if not os.path.exists(path):
                return
            if list(self.counters.keys()):
                lst = [int(time.monotonic()*1000)]
                for cnt in list(self.counters.keys()):
                    lst.append(self.counters[cnt])
                with open(path, 'a+', newline='') as csvfile:
                    csvwriter = csv.writer(csvfile, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    csvwriter.writerow(lst)

        def print_loggers(self):
            """
            Print the current value of all averagers registered.
            """
            if list(self.loggers.keys()):
                for logger in list(self.loggers.keys()):
                    path = self.subfolder + '/' + self.starttime + "_" + logger + '.csv'
                    if not os.path.exists(path):
                        continue
                    with open(path, 'a', newline='') as csvfile:
                        csvwriter = csv.writer(csvfile, delimiter=';', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                        for date in self.loggers[logger]:
                            csvwriter.writerow(date)
                        self.loggers[logger] = []

        def write(self):
            """
            Log all the collected data to csv files.
            """
            self.print_averagers()
            self.print_counters()
            self.print_loggers()

        def create_header(self, tname, line):
            """
            Create a header for the csv files.
            """
            try:
                os.makedirs(self.subfolder, exist_ok=True)
                fil = open(tname, 'w')
                fil.write(line)
                fil.close()
            except:
                logging.error(tname + ' could not be created')

        def run(self, queue, stats, subfolder):
            """
            Message processing of logger.
            """
            self.starttime = str(datetime.datetime.fromtimestamp(time.time()).strftime('%Y%m%d_%H%M%S'))
            self.subfolder = subfolder
            tmstp = time.monotonic() * 1000 + TARGET_SLEEP_TIME
            while True:
                try:
                    if tmstp < (time.monotonic() * 1000):
                        self.write()
                        tmstp = tmstp + 1000

                    msg = queue.get_nowait()

                    if msg.get_type() == stats.MessageType.STOP:
                        break
                    elif msg.get_type() == stats.MessageType.SUBFOLDER:
                        self.subfolder = msg.get_name()
                    elif msg.get_type() == stats.MessageType.STARTTIME:
                        pass
                    elif msg.get_type() == stats.MessageType.NEW_LOGGER:
                        self.loggers[msg.get_name()] = []
                        path = self.subfolder + '/' + self.starttime + "_" + msg.get_name() + '.csv'
                        self.create_header(path, "'timestamp';'value'\n")
                    elif msg.get_type() == stats.MessageType.NEW_COUNTER:
                        self.counters[msg.get_name()] = 0
                        path = self.subfolder + '/' + self.starttime + '_counters.csv'
                        self.create_header(path,
                                           '\'' + '\';\''.join(['timestamp'] + list(self.counters.keys())) + '\'\n')
                    elif msg.get_type() == stats.MessageType.NEW_AVERAGER:
                        self.create_header(self.subfolder + '/' + self.starttime + '_averagers.csv',
                                           '\'' + '\';\''.join(['timestamp'] + list(self.averagers.keys()) + msg.get_name()) + '\'\n')
                        self.averagers[msg.get_name()] = 0
                    elif msg.get_type() == stats.MessageType.NEW_LOGGER_VAL:
                        self.loggers[msg.get_name()].append([msg.get_timestamp(), msg.get_value()])
                    elif msg.get_type() == stats.MessageType.NEW_COUNTER_VAL:
                        self.counters[msg.get_name()] = msg.get_value()
                    elif msg.get_type() == stats.MessageType.NEW_AVERAGER_VAL:
                        self.averagers[msg.get_name()] = msg.get_value()

                except Empty:
                    pass
                except (KeyboardInterrupt, SystemExit):
                    logging.info('Received keyboard interrupt, writing files to disk.')
                    break
                except:
                    logging.info('Some other error')
                    break
            self.write()
            logging.info('Files written to disk.')

        def stop_logging(self, stats):
            """
            Stop recording of logs and close thread.
            """
            self.queue.put_nowait(stats.Message(stats.MessageType.STOP))
            logging.info("Trying to stop logging thread/process.")
            self.process.join()
            self.queue.close()
            self.queue.join_thread()
            logging.info("Process joined.")
            sys.exit(0)

    def __init__(self, subfolder='.'):
        self.queue = Queue()
        self.statistics_thread = self.StatisticsThread(self.queue, self, subfolder)

    class Counter():
        """
        Simple incremental counter.
        """
        def __init__(self, name, queue, stats):
            self.name = name
            self.value = 0
            self.queue = queue
            self.stats = stats
            self.queue.put_nowait(self.stats.Message(self.stats.MessageType.NEW_COUNTER, name=self.name))

        def inc(self, increase_by=1):
            """
            Increment counter.
            """
            self.value += increase_by
            self.queue.put_nowait(self.stats.Message(self.stats.MessageType.NEW_COUNTER_VAL, name=self.name, value=self.value))

    class Averager():
        """
        Average added values.
        """
        def __init__(self, name, queue, stats, weight=.2):
            self.name = name
            self.weight = weight
            self.val = 0
            self.queue = queue
            self.stats = stats
            self.queue.put_nowait(self.stats.Message(self.stats.MessageType.NEW_AVERAGER, name=self.name))

        def add_value(self, value):
            """
            Add current value and recalculate new average value.
            """
            self.val = value * self.weight + (1.0-self.weight) * self.val
            self.queue.put_nowait(self.stats.Message(self.stats.MessageType.NEW_AVERAGER_VAL, name=self.name, value=self.val))

    class TimestampLogger():
        """
        Log values with a timestamp.
        """
        def __init__(self, name, queue, stats):
            self.name = name
            self.queue = queue
            self.stats = stats
            self.queue.put_nowait(self.stats.Message(self.stats.MessageType.NEW_LOGGER, name=self.name))

        def timestamp(self, timestamp=-1, value=0):
            """
            Add new measurement value and timestamp.
            """
            if timestamp == -1:
                timestamp = int(time.monotonic() * 1000)
            self.queue.put_nowait(self.stats.Message(self.stats.MessageType.NEW_LOGGER_VAL,
                                                     name=self.name, value=value,
                                                     timestamp=timestamp))

    def new_averager(self, name):
        """
        Create a new averager class.
        """
        avg = self.Averager(name, self.queue, self)
        return avg

    def new_counter(self, name):
        """
        Create a new counter class.
        """
        cnt = self.Counter(name, self.queue, self)
        return cnt

    def new_logger(self, name):
        """
        Create a new timestamped logger.
        """
        tsl = self.TimestampLogger(name, self.queue, self)
        return tsl

    def stop_logging(self):
        """
        Stop the logger and kill logging thread.
        """
        self.statistics_thread.stop_logging(self)
