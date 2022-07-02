"""

Process_all.py:  Program will process all videos from given dataset and will measure processing time.

Authors: Ing. Jakub Spanhel, Jan Folenta
Email: xfolen00@stud.fit.vutbr.cz
Last update: 23.05.2020

"""

from subprocess import call
import time
import os

VIDEO_LIST = "../../datasets/AIC-2020/list_video_id.txt"
OUTPUT_CACHE = "outputs"
TIME_OUTPUT_FILE = "time_all.txt"

if __name__ == "__main__":
    total_start_time = time.time()

    output_file = open(os.path.join(OUTPUT_CACHE,TIME_OUTPUT_FILE), "w")

    video_files = None
    with open(VIDEO_LIST, "r") as f:
        video_files = [line.strip() for line in f.readlines()]


    for video_file in video_files:
        start_time = time.time()
        video_id, video_name = video_file.split(" ")[:2]

        print("Processing file: ", video_id, video_name)
        call(['python', 'process_video.py', video_name[:-4]])
        end_time = time.time()
        output_file.write("Video id: %2d - %s - time: %.4f\n" % (int(video_id), video_name, (end_time - start_time)))

    output_file.write("Total processing time: %.4f" % (end_time - total_start_time))
    output_file.close()

    output_file = open(os.path.join(OUTPUT_CACHE, "text", "results_all.txt"), "w")

    video_files = None
    with open(VIDEO_LIST, "r") as f:
        video_files = [line.strip() for line in f.readlines()]

    for video_file in video_files:
        start_time = time.time()
        video_id, video_name = video_file.split(" ")[:2]

        output_data = None
        # find output video
        with open(os.path.join(OUTPUT_CACHE, "text", "%s_out.txt" % (video_name[:-4]))) as f:
            output_data = [line.strip() for line in f.readlines()]
            for line in output_data:
                output_file.write("%s\n" % line)

            output_file.flush()
    output_file.close()
