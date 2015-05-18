#include <linux/percpu.h>

#include <asm/cacheflush.h>

#define FLUSH_CLEAN_BY_SET_WAY_PAGE_THRESHOLD 8
extern void __flush_dcache_page(struct address_space *, struct page *);

static inline void __flush_cache_all(void *info)
{
	flush_cache_all();
}

inline void inner_flush_cache_all(void)
{
	on_each_cpu(__flush_cache_all, NULL, 1);
}

void update_page_count(int level, unsigned long pages)
{
}

static void flush_cache(struct page **pages, int numpages)
{
	unsigned int i;
	bool flush_inner = true;
	unsigned long base;

	if (numpages >= FLUSH_CLEAN_BY_SET_WAY_PAGE_THRESHOLD) {
		inner_flush_cache_all();
		flush_inner = false;
	}

	for (i = 0; i < numpages; i++) {
		if (flush_inner)
			__flush_dcache_page(page_mapping(pages[i]), pages[i]);
		base = page_to_phys(pages[i]);
		outer_flush_range(base, base + PAGE_SIZE);
	}
}

int set_pages_array_uc(struct page **pages, int addrinarray)
{
	flush_cache(pages, addrinarray);
	return 0;
}
EXPORT_SYMBOL(set_pages_array_uc);

int set_pages_array_wc(struct page **pages, int addrinarray)
{
	flush_cache(pages, addrinarray);
	return 0;
}
EXPORT_SYMBOL(set_pages_array_wc);

int set_pages_array_wb(struct page **pages, int addrinarray)
{
	return 0;
}
EXPORT_SYMBOL(set_pages_array_wb);

int set_pages_array_iwb(struct page **pages, int addrinarray)
{
	flush_cache(pages, addrinarray);
	return 0;
}
EXPORT_SYMBOL(set_pages_array_iwb);
