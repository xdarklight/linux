#ifndef __OF_IEEE80211_H
#define __OF_IEEE80211_H

struct device_node;

#ifdef CONFIG_OF_IEEE80211

bool of_ieee80211_is_2ghz_enabled(struct device_node *np);
bool of_ieee80211_is_5ghz_enabled(struct device_node *np);

bool of_ieee80211_is_2ghz_disabled(struct device_node *np);
bool of_ieee80211_is_5ghz_disabled(struct device_node *np);

#else /* CONFIG_OF_IEEE80211 */

static inline bool of_ieee80211_is_2ghz_enabled(struct device_node *np)
{
	return false;
}

static inline bool of_ieee80211_is_5ghz_enabled(struct device_node *np)
{
	return false;
}

static inline bool of_ieee80211_is_2ghz_disabled(struct device_node *np)
{
	return false;
}

static inline bool of_ieee80211_is_5ghz_disabled(struct device_node *np)
{
	return false;
}

#endif /* CONFIG_OF_IEEE80211 */

#endif /* __OF_IEEE80211_H */
