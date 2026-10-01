#include "./lv_i18n.h"


////////////////////////////////////////////////////////////////////////////////
// Define plural operands
// http://unicode.org/reports/tr35/tr35-numbers.html#Operands

// Integer version, simplified

#define UNUSED(x) (void)(x)

static inline uint32_t op_n(int32_t val) { return (uint32_t)(val < 0 ? -val : val); }
static inline uint32_t op_i(uint32_t val) { return val; }
// always zero, when decimal part not exists.
static inline uint32_t op_v(uint32_t val) { UNUSED(val); return 0;}
static inline uint32_t op_w(uint32_t val) { UNUSED(val); return 0; }
static inline uint32_t op_f(uint32_t val) { UNUSED(val); return 0; }
static inline uint32_t op_t(uint32_t val) { UNUSED(val); return 0; }

static lv_i18n_phrase_t en_singulars[] = {
    {"Screen Brightness :", "Screen Brightness :"},
    {"Enable key sound", "Enable key sound"},
    {"Enable Chinese", "Enable Chinese"},
    {"Chose Rotation", "Chose Rotation"},
    {"Current Range", "Current Range"},
    {"AUTO", "AUTO"},
    {"LOW", "LOW"},
    {"MID", "MID"},
    {"HIGH", "HIGH"},
    {"Please reboot to apply\nall changes!", "Please reboot to apply\nall changes!"},
    {NULL, NULL} // End mark
};



static uint8_t en_plural_fn(int32_t num)
{
    uint32_t n = op_n(num); UNUSED(n);
    uint32_t i = op_i(n); UNUSED(i);
    uint32_t v = op_v(n); UNUSED(v);

    if ((i == 1 && v == 0)) return LV_I18N_PLURAL_TYPE_ONE;
    return LV_I18N_PLURAL_TYPE_OTHER;
}

static const lv_i18n_lang_t en_lang = {
    .locale_name = "en",
    .singulars = en_singulars,

    .locale_plural_fn = en_plural_fn
};

static lv_i18n_phrase_t zh_cn_singulars[] = {
    {"Screen Brightness :", "\xE5\xB1\x8F\xE5\xB9\x95\xE4\xBA\xAE\xE5\xBA\xA6\x20\x3A"},
    {"Enable key sound", "\xE6\x8C\x89\xE9\x94\xAE\xE5\xA3\xB0\xE9\x9F\xB3"},
    {"Enable Chinese", "\xE4\xB8\xAD\xE8\x8B\xB1\xE6\x96\x87\xE5\x88\x87\xE6\x8D\xA2"},
    {"Chose Rotation", "\xE6\x97\x8B\xE8\xBD\xAC\xE8\xA7\x92\xE5\xBA\xA6\x20\x3A"},
    {"Current Range", "\xE5\xBD\x93\xE5\x89\x8D\xE9\x87\x8F\xE7\xA8\x8B\x20\x3A"},
    {"AUTO", "\xE8\x87\xAA\xE5\x8A\xA8"},
    {"LOW", "\xE4\xBD\x8E\xE6\xA1\xA3"},
    {"MID", "\xE4\xB8\xAD\xE6\xA1\xA3"},
    {"HIGH", "\xE9\xAB\x98\xE6\xA1\xA3"},
    {"Please reboot to apply\nall changes!", "\xE8\xAF\xB7\xE9\x87\x8D\xE5\x90\xAF\xE4\xBB\xA5\xE5\xBA\x94\xE7\x94\xA8\n\xE6\x89\x80\xE6\x9C\x89\xE6\x9B\xB4\xE6\x94\xB9\x21"},
    {NULL, NULL} // End mark
};



static uint8_t zh_cn_plural_fn(int32_t num)
{



    return LV_I18N_PLURAL_TYPE_OTHER;
}

static const lv_i18n_lang_t zh_cn_lang = {
    .locale_name = "zh-cn",
    .singulars = zh_cn_singulars,

    .locale_plural_fn = zh_cn_plural_fn
};

const lv_i18n_language_pack_t lv_i18n_language_pack[] = {
    &en_lang,
    &zh_cn_lang,
    NULL // End mark
};

////////////////////////////////////////////////////////////////////////////////


// Internal state
static const lv_i18n_language_pack_t * current_lang_pack;
static const lv_i18n_lang_t * current_lang;


/**
 * Reset internal state. For testing.
 */
void __lv_i18n_reset(void)
{
    current_lang_pack = NULL;
    current_lang = NULL;
}

/**
 * Set the languages for internationalization
 * @param langs pointer to the array of languages. (Last element has to be `NULL`)
 */
int lv_i18n_init(const lv_i18n_language_pack_t * langs)
{
    if(langs == NULL) return -1;
    if(langs[0] == NULL) return -1;

    current_lang_pack = langs;
    current_lang = langs[0];     /*Automatically select the first language*/
    return 0;
}

/**
 * Change the localization (language)
 * @param l_name name of the translation locale to use. E.g. "en-GB"
 */
int lv_i18n_set_locale(const char * l_name)
{
    if(current_lang_pack == NULL) return -1;

    uint16_t i;

    for(i = 0; current_lang_pack[i] != NULL; i++) {
        // Found -> finish
        if(strcmp(current_lang_pack[i]->locale_name, l_name) == 0) {
            current_lang = current_lang_pack[i];
            return 0;
        }
    }

    return -1;
}


static const char * __lv_i18n_get_text_core(lv_i18n_phrase_t * trans, const char * msg_id)
{
    uint16_t i;
    for(i = 0; trans[i].msg_id != NULL; i++) {
        if(strcmp(trans[i].msg_id, msg_id) == 0) {
            /*The msg_id has found. Check the translation*/
            if(trans[i].translation) return trans[i].translation;
        }
    }

    return NULL;
}


/**
 * Get the translation from a message ID
 * @param msg_id message ID
 * @return the translation of `msg_id` on the set local
 */
const char * lv_i18n_get_text(const char * msg_id)
{
    if(current_lang == NULL) return msg_id;

    const lv_i18n_lang_t * lang = current_lang;
    const void * txt;

    // Search in current locale
    if(lang->singulars != NULL) {
        txt = __lv_i18n_get_text_core(lang->singulars, msg_id);
        if (txt != NULL) return txt;
    }

    // Try to fallback
    if(lang == current_lang_pack[0]) return msg_id;
    lang = current_lang_pack[0];

    // Repeat search for default locale
    if(lang->singulars != NULL) {
        txt = __lv_i18n_get_text_core(lang->singulars, msg_id);
        if (txt != NULL) return txt;
    }

    return msg_id;
}

/**
 * Get the translation from a message ID and apply the language's plural rule to get correct form
 * @param msg_id message ID
 * @param num an integer to select the correct plural form
 * @return the translation of `msg_id` on the set local
 */
const char * lv_i18n_get_text_plural(const char * msg_id, int32_t num)
{
    if(current_lang == NULL) return msg_id;

    const lv_i18n_lang_t * lang = current_lang;
    const void * txt;
    lv_i18n_plural_type_t ptype;

    // Search in current locale
    if(lang->locale_plural_fn != NULL) {
        ptype = lang->locale_plural_fn(num);

        if(lang->plurals[ptype] != NULL) {
            txt = __lv_i18n_get_text_core(lang->plurals[ptype], msg_id);
            if (txt != NULL) return txt;
        }
    }

    // Try to fallback
    if(lang == current_lang_pack[0]) return msg_id;
    lang = current_lang_pack[0];

    // Repeat search for default locale
    if(lang->locale_plural_fn != NULL) {
        ptype = lang->locale_plural_fn(num);

        if(lang->plurals[ptype] != NULL) {
            txt = __lv_i18n_get_text_core(lang->plurals[ptype], msg_id);
            if (txt != NULL) return txt;
        }
    }

    return msg_id;
}

/**
 * Get the name of the currently used locale.
 * @return name of the currently used locale. E.g. "en-GB"
 */
const char * lv_i18n_get_current_locale(void)
{
    if(!current_lang) return NULL;
    return current_lang->locale_name;
}
