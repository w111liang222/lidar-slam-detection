import React, { useEffect, useRef, useCallback } from "react";

interface Props {
  callback: (entries: ResizeObserverEntry[]) => void;
  element: React.RefObject<HTMLElement>;
  delay?: number;
}

export const useResizeObserver = ({ callback, element, delay = 0 }: Props, deps: any[] = []): (() => void) => {
  const current = element && element.current;
  const timeout = useRef<NodeJS.Timeout>();

  const observer = useRef<any>(null);

  const observe = useCallback(() => {
    if (element && element.current && observer.current) {
      observer.current.observe(element.current);
    }
  }, [element]);

  const unObserve = useCallback(() => {
    if (observer && observer.current && element && element.current) {
      observer.current.unobserve(element.current);
    }
  }, [observer, element]);

  useEffect(() => {
    const withDebounce = (entries: ResizeObserverEntry[]) => {
      clearTimeout(timeout.current as unknown as number);
      timeout.current = setTimeout(() => {
        callback(entries);
      }, delay);
    };

    // if we are already observing old element
    if (observer && observer.current && current) {
      observer.current.unobserve(current);
    }
    observer.current = new ResizeObserver(withDebounce);
    observe();

    return () => {
      unObserve();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [current, element, observe, unObserve, callback, delay, ...deps]);

  return unObserve;
};
