#!/usr/bin/env python3

import argparse
import sys
import logging
from pathlib import Path

import cv2

def extract_frames(video, frames=None, outdir='.', fname_gen=None):
    '''Extract frames from the video and save to file.

    @param video: path to video
    @param frames: None or list of frame numbers to extract
        If None, then will extract all frames
    @param outdir: directory to output frames.  Default is '.'.
    @param fname_gen: function taking frame number as the parameter and
        returning a filename, which will be appended to outdir (may contain
        subdirectories if desired).  Default is 'frame-{:05d}.jpg'.format.
    '''
    logging.debug(f'extract_frames(video={video}, frames={frames}, outdir={outdir})')
    if fname_gen is None:
        fname_gen = 'frame-{:05d}.jpg'.format
    if frames is not None:
        frames = sorted(frames, reverse=True)
        logging.debug(f'frames = {frames}')
    outdir = Path(outdir)

    cap = cv2.VideoCapture(video)
    frameno = -1
    while cap.isOpened():
        #success, frame = cap.read() # same as grab() and retrieve()
        success = cap.grab() # grab the next frame
        if not success:
            logging.info(f'Finished reading at frame {frameno}')
            break
        frameno += 1 # update frame number
        if frames is not None and frameno not in frames:
            logging.debug(f'Skipping frame {frameno}')
            continue

        # output frame
        success, frame = cap.retrieve()  # encode the last grabbed frame
        if not success:
            logging.warning(f'Could not retrieve frame {frameno}')
            break
        fpath = outdir / fname_gen(frameno)
        logging.info(f'Writing {fpath}')
        cv2.imwrite(str(fpath), frame)

        if frames is not None:
            popped = frames.pop()
            assert popped == frameno, (popped, frameno)
            if not frames:
                logging.info(f'Reached last requested frame, closing video')
                break

    cap.release()
    cv2.destroyAllWindows()

def populate_parser(parser=None):
    if parser is None:
        parser = argparse.ArgumentParser()
    parser.description = 'Extract frames from the video and save to file'
    parser.formatter_class = argparse.ArgumentDefaultsHelpFormatter
    parser.add_argument('video', help='video file.  Any type that opencv can open')
    parser.add_argument('-o', '--outdir', default='.', help='output directory')
    parser.add_argument('-f', '--frames', default='all',
                        help='''
                            Comma-separated list of frames to extract.  By
                            default, will extract all frames.''')
    parser.add_argument('-l', '--loglevel', default='INFO',
                        help='Set logging level to this (from logging module)')
    parser.add_argument('-s', '--suffix', default='.jpg',
                        help='''
                            Image suffix to use for output frames.  This should
                            contain the file extension, as the output image
                            type will be based on the file extension.
                            ''')
    parser.add_argument('-p', '--prefix', default='frame-',
                        help='frame file name prefix (before frame number)')
    return parser

def parse_args(parser, arguments):
    args = parser.parse_args(arguments)
    args.loglevel = getattr(logging, args.loglevel.upper())
    if args.frames == 'all':
        args.frames = None
    else:
        args.frames = [int(x) for x in args.frames.split(',')]
    return args

def main(arguments):
    parser = populate_parser()
    args = parse_args(parser, arguments)
    logging.basicConfig(level=args.loglevel)
    logging.debug(f'Arguments: {arguments}')
    logging.debug(f'Parsed args: {args}')
    extract_frames(
        video=args.video,
        frames=args.frames,
        outdir=args.outdir,
        fname_gen=f'{args.prefix}{{:05d}}{args.suffix}'.format,
    )

if __name__ == '__main__':
    main(sys.argv[1:])
