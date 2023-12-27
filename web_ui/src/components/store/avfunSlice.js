import WEB_STORE from "@rpc/sample/webstore.json";
import { createSlice } from "@reduxjs/toolkit";
import { getAvfuns } from "@rpc/http";
import _ from "lodash";

export const avfunSlice = createSlice({
  name: "avfuns",
  initialState: WEB_STORE.avfuns,
  reducers: {
    getAvfunsConfigAction: (state, action) => {
      return action.payload;
    },
  },
});

export const { getAvfunsConfigAction } = avfunSlice.actions;
export default avfunSlice.reducer;

export const getAvfunsConfig = () => {
  return async (dispatch) => {
    try {
      let avfuns = await getAvfuns();
      if (JSON.stringify(avfuns) === "{}") {
        avfuns = WEB_STORE.avfuns;
      }
      avfuns = _.merge(JSON.parse(JSON.stringify(WEB_STORE.avfuns)), JSON.parse(JSON.stringify(avfuns)));
      dispatch(getAvfunsConfigAction(avfuns));
    } catch (e) {
      dispatch(getAvfunsConfigAction(WEB_STORE.avfuns));
      console.log(e);
    }
  };
};
