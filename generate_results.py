from subprocess import call
import time
import os

VIDEO_LIST = "/mnt/data/AIC2020/AIC20_track1/Dataset_A/list_video_id.txt"
OUTPUT_ROOT = "/mnt/data/AIC2020-counting-cache/outlist"


if __name__ == "__main__":
    total_start_time = time.time()


    output_file = open(os.path.join(OUTPUT_ROOT,"results_all.txt"), "w")


    video_files = None
    with open(VIDEO_LIST, "r") as f:
        video_files = [line.strip() for line in f.readlines()]

    # for i in range(1,41):
    for video_file in video_files:
        start_time = time.time()
        video_id, video_name = video_file.split(" ")[:2]

        output_data = None
        # find output video
        with open(os.path.join(OUTPUT_ROOT,"%s_out.txt" % (video_name[:-4]))) as f:
            output_data = [line.strip() for line in f.readlines()]
            for line in output_data:
                if line > "":
                    output_file.write("%s\n" % line.replace(","," "))

            output_file.flush()
    output_file.close()