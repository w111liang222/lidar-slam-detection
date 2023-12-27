type KeyExists<U, U1> = [U1] extends [keyof U] ? true : false;
type TObjKey<O extends object, K extends keyof O> = KeyExists<O, K> extends true ? O[K] : unknown;
type TObj<O, K extends keyof O> = O extends object
  ? {
      [key in K]: TObjKey<O, key>;
    }
  : {
      [key in K]: unknown;
    } & {
      [key: string]: unknown;
    };

/**
 * Checks if an object has properties
 * @param potentialObj
 * @returns
 */
export function objHasProp<O, K extends string>(
  potentialObj: O,
  keys: K[]
  // @ts-ignore
): potentialObj is TObj<O, K> {
  if (
    typeof potentialObj !== "object" ||
    potentialObj === null ||
    potentialObj instanceof Date ||
    potentialObj instanceof Array
  )
    return false;

  if (keys.every((key) => key in potentialObj)) {
    return true;
  }
  return false;
}
