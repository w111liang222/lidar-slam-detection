// plugins
import * as yup from "yup";
function ipv4(message = "Invalid IP address") {
  return this.matches(/(^(\d{1,3}\.){3}(\d{1,3})$)/, {
    message,
    excludeEmptyString: true,
  }).test("ip", message, (value) => {
    return value === undefined || value.trim() === ""
      ? true
      : value.split(".").find((i) => parseInt(i, 10) > 255) === undefined;
  });
}
yup.addMethod(yup.string, "ipv4", ipv4);

yup.addMethod(yup.object, "uniqueProperty", function (propertyName, message) {
  return this.test("unique", message, function (value) {
    if (!value || !value[propertyName]) {
      return true;
    }

    if (this.parent.filter((v) => v !== value).some((v) => v[propertyName] === value[propertyName])) {
      throw this.createError({
        path: `${this.path}.${propertyName}`,
      });
    }

    return true;
  });
});

yup.addMethod(yup.number, "elementLimit", function (min, max, message) {
  return this.test("limit", message, function (value) {
    let i = Number(this.path[this.path.length - 2]);
    if (value < min[i] || value > max[i]) {
      throw this.createError({
        path: `${this.path}`,
        message: message[i],
      });
    }
    return true;
  });
});

export default yup;
