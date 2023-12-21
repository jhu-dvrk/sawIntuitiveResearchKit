#!/usr/bin/env python3

#
# Script used to find and remove cisstLog files
#
# Anton Deguet
#

import argparse
import glob
import os

def readable_size(size, decimal_point = 2):
   for i in ['B', 'KB', 'MB', 'GB', 'TB']:
      if size < 1024.0:
         break
      size /= 1024.0
   return f'{size:.{decimal_point}f} {i}'


def remove_files(files, what, skip_prompt = False):
   if not files:
      print(f'No {what} files found')
      return

   print(f'Removing {what} files')

   total_size = 0
   for file in files:
      stats = os.stat(file)
      total_size += stats.st_size

   while True:
      answer = 'd'
      if not skip_prompt:
         print(f'Total size of {what} file found {readable_size(total_size)}')
         answer = input('[s] to show the files, [d] to delete them, [q] to quit\n')
      if answer == 's':
         for file in files:
            stats = os.stat(file)
            print(f'{file}   {readable_size(stats.st_size)}')
      elif answer == 'd':
         for file in files:
            print(f'Removing {file}')
            os.remove(file)
         return
      elif answer == 'q':
         return

def main():
   parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
   parser.add_argument(
      "-f",
      "--force",
      help = "force deletion without prompting user",
      action = 'store_true'
   )
   args = parser.parse_args()
   if not args.force:
      print('You can use the command line argument -f, --force to avoid the prompts')

   # logs
   files = glob.glob('./**/cisstLog.txt', recursive = True) \
      + glob.glob('./**/cisstLog-20*-*.txt', recursive = True)
   remove_files(files, 'log', skip_prompt = args.force)

   # backups and extra files from calibration steps
   files = glob.glob('./**/sawRobotIO*.xml-backup*', recursive = True) \
      + glob.glob('./**/*.json-backup-20*', recursive = True) \
      + glob.glob('./**/pot_calib_offsets_*.csv', recursive = True) \
      + glob.glob('./**/pot_calib_scales_*.csv', recursive = True)
   remove_files(files, 'backup', skip_prompt = args.force)

if __name__ == "__main__":
   main()
