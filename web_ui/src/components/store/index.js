import { configureStore } from "@reduxjs/toolkit";
import avfunsReducer from "./avfunSlice";

export default configureStore({
  reducer: {
    avfuns: avfunsReducer,
  },
});
