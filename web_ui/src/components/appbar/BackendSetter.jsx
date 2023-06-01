import React from "react";
import { HOSTNAME, axios } from "@rpc/http";
import { axios as upgrade_axios } from "@rpc/http-upgrade";

export default function BackendSetter() {
  return (
    <input
      defaultValue={HOSTNAME}
      onBlur={(ev) => {
        console.log(ev.target.value);
        axios.defaults.baseURL = `http://${ev.target.value}:80/`;
        upgrade_axios.defaults.baseURL = `http://${ev.target.value}:1235/`;
      }}
    />
  );
}
