#include <linux/clk.h>
#include <linux/clk-provider.h>

int tegra_is_clk_enabled(struct clk *c)
{
	return __clk_get_enable_count(c);
}
EXPORT_SYMBOL(tegra_is_clk_enabled);
