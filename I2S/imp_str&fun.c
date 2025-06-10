/*This structure is passed by ALSA PCM layer to ALSA subsystem*/

struct _snd_pcm {
	void *open_func;
	char *name;
	snd_pcm_type_t type;
	snd_pcm_stream_t stream;
	int mode;
	long minperiodtime;		/* in us */
	int poll_fd_count;
	int poll_fd;
	unsigned short poll_events;
	int setup: 1,
	    compat: 1;
	snd_pcm_access_t access;	/* access mode */
	snd_pcm_format_t format;	/* SND_PCM_FORMAT_* */
	snd_pcm_subformat_t subformat;	/* subformat */
	unsigned int channels;		/* channels */
	unsigned int rate;		/* rate in Hz */
	snd_pcm_uframes_t period_size;
	unsigned int period_time;	/* period duration */
	snd_interval_t periods;
	snd_pcm_tstamp_t tstamp_mode;	/* timestamp mode */
	snd_pcm_tstamp_type_t tstamp_type;	/* timestamp type */
	unsigned int period_step;
	snd_pcm_uframes_t avail_min;	/* min avail frames for wakeup */
	int period_event;
	snd_pcm_uframes_t start_threshold;
	snd_pcm_uframes_t stop_threshold;
	snd_pcm_uframes_t silence_threshold;	/* Silence filling happens when
					   noise is nearest than this */
	snd_pcm_uframes_t silence_size;	/* Silence filling size */
	snd_pcm_uframes_t boundary;	/* pointers wrap point */
	unsigned int info;		/* Info for returned setup */
	unsigned int msbits;		/* used most significant bits */
	unsigned int rate_num;		/* rate numerator */
	unsigned int rate_den;		/* rate denominator */
	unsigned int hw_flags;		/* actual hardware flags */
	snd_pcm_uframes_t fifo_size;	/* chip FIFO size in frames */
	snd_pcm_uframes_t buffer_size;
	snd_interval_t buffer_time;
	unsigned int sample_bits;
	unsigned int frame_bits;
	snd_pcm_rbptr_t appl;
	snd_pcm_rbptr_t hw;
	snd_pcm_uframes_t min_align;
	unsigned int mmap_rw: 1;	/* use always mmapped buffer */
	unsigned int mmap_shadow: 1;	/* don't call actual mmap,
					 * use the mmaped buffer of the slave
					 */
	unsigned int donot_close: 1;	/* don't close this PCM */
	unsigned int own_state_check:1; /* plugin has own PCM state check */
	snd_pcm_channel_info_t *mmap_channels;
	snd_pcm_channel_area_t *running_areas;
	snd_pcm_channel_area_t *stopped_areas;
	const snd_pcm_ops_t *ops;
	const snd_pcm_fast_ops_t *fast_ops;
	snd_pcm_t *op_arg;
	snd_pcm_t *fast_op_arg;
	void *private_data;
	struct list_head async_handlers;
#ifdef THREAD_SAFE_API
	int need_lock;		/* true = this PCM (plugin) is thread-unsafe,
				 * thus it needs a lock.
				 */
	int lock_enabled;	/* thread-safety lock is enabled on the system;
				 * it's set depending on $LIBASOUND_THREAD_SAFE.
				 */
	pthread_mutex_t lock;
#endif
};