#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>

#define PCM_DEVICE "default"  // ALSA PCM device, use 'default' if unsure

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <file.wav>\n", argv[0]);
        return 1;
    }

    const char *filename = argv[1];
    FILE *wav_file = fopen(filename, "rb");
    if (!wav_file) {
        perror("Error opening file");
        return 1;
    } else {
        printf("File opened successfully\n");
    }

    // WAV header parsing (simplified)
    char buffer[44];
    fread(buffer, 1, 44, wav_file); // Skip WAV header
    unsigned int rate = *(unsigned int *)&buffer[24]; // Sample rate
    unsigned short channels = *(unsigned short *)&buffer[22]; // Number of channels
    unsigned short bits_per_sample = *(unsigned short *)&buffer[34]; // Bits per sample
	
    // ALSA variables
    snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *params;
    int pcm, dir;
    unsigned int exact_rate = rate;
    snd_pcm_uframes_t frames = 32;

    // Open the PCM device
    pcm = snd_pcm_open(&pcm_handle, PCM_DEVICE, SND_PCM_STREAM_PLAYBACK, 0);
    if (pcm < 0) {
        fprintf(stderr, "Error opening PCM device: %s\n", snd_strerror(pcm));
        return 1;
    }
    else printf("snd_pcm_open:sucessfully");

    // Allocate hardware parameters
    snd_pcm_hw_params_malloc(&params); //allocating memory
    snd_pcm_hw_params_any(pcm_handle, params); //initializes structure params

    // Set hardware parameters
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (bits_per_sample == 16) {
        snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);
    } else if (bits_per_sample == 8) {
        snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_U8);
    } else {
        fprintf(stderr, "Unsupported bits per sample: %d\n", bits_per_sample);
        return 1;
    }
    snd_pcm_hw_params_set_channels(pcm_handle, params, channels);
    snd_pcm_hw_params_set_rate_near(pcm_handle, params, &exact_rate, &dir);
    snd_pcm_hw_params_set_period_size_near(pcm_handle, params, &frames, &dir);

    // Apply parameters
    pcm = snd_pcm_hw_params(pcm_handle, params);
    if (pcm < 0) {
        fprintf(stderr, "Error setting HW parameters: %s\n", snd_strerror(pcm));
        return 1;
    }

    // Buffer for audio data
    int size = frames * channels * (bits_per_sample / 8); // Frame size
    char *data = (char *)malloc(size);

    // Playback loop
    while (fread(data, 1, size, wav_file) > 0) {
        pcm = snd_pcm_writei(pcm_handle, data, frames);
        if (pcm == -EPIPE) {
            fprintf(stderr, "XRUN.\n");
            snd_pcm_prepare(pcm_handle);
        } else if (pcm == -EAGAIN) {
            continue;
        } else if (pcm < 0) {
            fprintf(stderr, "Error writing to PCM device: %s\n", snd_strerror(pcm));
        }
    }

    // Clean up
    free(data);
    snd_pcm_drain(pcm_handle);
    snd_pcm_close(pcm_handle);
    fclose(wav_file);

    return 0;
}