import i18n from "i18next";
import { initReactI18next } from "react-i18next";
import translationEn from "./i18n-en";
import translationZh from "./i18n-zh";

// the translations
// (tip move them in a JSON file and import them)
export const resources = {
  "zh-CN": {
    translation: translationZh,
  },
  "en-US": {
    translation: translationEn,
  },
};

i18n
  .use(initReactI18next) // passes i18n down to react-i18next
  .init({
    resources,
    lng: "zh-CN",

    keySeparator: false, // we do not use keys in form messages.welcome

    interpolation: {
      escapeValue: false, // react already safes from xss
    },
  });

export default i18n;
