import React, { useCallback, useEffect, useRef, useState } from "react";
import { CategoricalChartState } from "recharts/types/chart/generateCategoricalChart";
import { objHasProp } from "./utils";
import { useResizeObserver } from "./useResizeObserver";

const CHART_AXIS_CLIP_PADDING = 50;

const CHART_CLASSES = {
  xAxis: "xAxis",
  grid: "recharts-cartesian-grid",
  line: "chart-line",
};

const getZoomValues = (
  mousePosition: { x: number; width: number } | null,
  edgeTolerance = 0.05,
  zoomCoefficient = 0.25
) => {
  if (!mousePosition) {
    return { zoomLeft: 1, zoomRight: 1 };
  }

  const { x, width } = mousePosition;
  const zoomCoef = width * zoomCoefficient;
  let xToWidth = x / width;
  if (xToWidth <= edgeTolerance) {
    xToWidth = 0;
  } else if (xToWidth >= 1 - edgeTolerance) {
    xToWidth = 1;
  }

  const zoomLeft = xToWidth * zoomCoef;
  const zoomRight = zoomCoef - zoomLeft;
  return { zoomLeft, zoomRight };
};

export const useZoomAndPan = ({
  chartAxisClipPadding = CHART_AXIS_CLIP_PADDING,
  chartLoaded,
}: {
  chartAxisClipPadding?: number;
  chartLoaded: boolean;
}) => {
  const wrapperRef = useRef<null | HTMLElement>(null);
  const clipPathRefs = useRef<{
    grid: React.MutableRefObject<SVGRectElement | null>;
    axis: React.MutableRefObject<SVGRectElement | null>;
  } | null>(null);
  const gridRef = useRef<SVGSVGElement | null>(null);
  const chartMouseDown = useRef<{ x: number; y: number } | null>(null);
  const chartXPaddingOnMouseDown = useRef<[number, number] | null>(null);
  const [xPadding, setXPadding] = useState<[number, number]>([0, 0]);
  const [mousePositionToGrid, setMousePositionToGrid] = useState<{
    x: number;
    width: number;
  } | null>(null);

  const setWrapperRef = useCallback((e: unknown) => {
    if (typeof e === "object" && e !== null && objHasProp(e, ["current"]) && e.current instanceof HTMLElement) {
      wrapperRef.current = e.current;
    }
  }, []);

  const setClipPaths = useCallback(
    (xAxis: SVGSVGElement) => {
      if (
        wrapperRef.current &&
        gridRef.current &&
        clipPathRefs?.current?.axis?.current &&
        clipPathRefs?.current?.grid?.current
      ) {
        const wrapperRect = wrapperRef.current.getBoundingClientRect();
        const gridRect = gridRef.current.getBoundingClientRect();
        console.log(wrapperRect);
        clipPathRefs.current.axis.current.setAttribute("width", `${gridRect.width + chartAxisClipPadding}px`);
        clipPathRefs.current.axis.current.style.transform = `translateX(${
          gridRect.x - wrapperRect.x - chartAxisClipPadding / 2
        }px)`;

        clipPathRefs.current.grid.current.setAttribute("width", `${gridRect.width}px`);
        clipPathRefs.current.grid.current.style.transform = `translateX(${gridRect.x - wrapperRect.x}px)`;

        gridRef.current?.setAttribute("clip-path", "url(#chart-grid-clip)");
        xAxis.setAttribute("clip-path", "url(#chart-xaxis-clip)");
      }
    },
    [chartAxisClipPadding]
  );

  const resizeObserverCallback = useCallback(
    (e) => {
      console.log(e);
      if (wrapperRef.current) {
        const xAxis = wrapperRef.current.querySelector(`.${CHART_CLASSES.xAxis}`) as SVGSVGElement | null;
        if (xAxis) {
          setClipPaths(xAxis);
        }
      }
    },
    [setClipPaths]
  );

  const unobserve = useResizeObserver({
    element: wrapperRef,
    callback: resizeObserverCallback,
    delay: 100,
  });

  useEffect(() => () => unobserve());

  const chartPan = (state: CategoricalChartState, e: MouseEvent) => {
    if (chartMouseDown.current !== null && state?.chartX && state?.chartY && chartXPaddingOnMouseDown.current) {
      const xDistance = chartMouseDown.current.x - state.chartX;
      const [paddingLeft, paddingRight] = chartXPaddingOnMouseDown.current;

      const panPaddingLeft = paddingLeft - xDistance;
      const panPaddingRight = paddingRight + xDistance;

      if (panPaddingLeft > 0) {
        setXPadding(([, pr]) => [0, pr]);
        return;
      }
      if (panPaddingRight > 0) {
        setXPadding(([pl]) => [pl, 0]);
        return;
      }
      setXPadding([Math.min(paddingLeft - xDistance, 0), Math.min(paddingRight + xDistance, 0)]);
    }
  };

  const onChartMouseMove = useCallback((state: CategoricalChartState, e: MouseEvent) => {
    const target = e.target as HTMLElement | null;
    if (chartMouseDown.current !== null) chartPan(state, e);
    if (target && clipPathRefs?.current?.axis?.current) {
      const { width, left } = clipPathRefs.current.axis.current.getBoundingClientRect();
      const x = Math.min(Math.max(e.clientX - left, 0), width);
      setMousePositionToGrid((state) => {
        if (!state?.width) return { x, width };
        return {
          ...state,
          x,
        };
      });
    }
  }, []);

  const onChartMouseDown = useCallback(
    (state: CategoricalChartState, e: MouseEvent) => {
      if (state) {
        const { chartX, chartY } = state;
        if (typeof chartX === "number" && typeof chartY === "number") {
          chartMouseDown.current = { x: chartX, y: chartY };
          chartXPaddingOnMouseDown.current = xPadding;
        }
      }
    },
    [xPadding]
  );

  const onChartMouseUp = useCallback(() => {
    chartMouseDown.current = null;
    chartXPaddingOnMouseDown.current = null;
  }, []);

  useEffect(() => {
    if (chartLoaded && wrapperRef.current) {
      const grid = wrapperRef.current.querySelector(`.${CHART_CLASSES.grid}`) as SVGSVGElement | null;

      const xAxis = wrapperRef.current.querySelector(`.${CHART_CLASSES.xAxis}`) as SVGSVGElement | null;
      gridRef.current = grid;
      if (xAxis) setClipPaths(xAxis);
    }
  }, [chartLoaded, setClipPaths]);

  const zoomOut = useCallback(() => {
    setXPadding((p) => {
      const [left, right] = p;
      const { zoomRight, zoomLeft } = getZoomValues(mousePositionToGrid);
      return [Math.min(left + zoomLeft, 0), Math.min(right + zoomRight, 0)];
    });
  }, [mousePositionToGrid]);

  const zoomIn = useCallback(() => {
    setXPadding((p) => {
      const [left, right] = p;
      const { zoomRight, zoomLeft } = getZoomValues(mousePositionToGrid);
      return [left - zoomLeft, right - zoomRight];
    });
  }, [mousePositionToGrid]);

  useEffect(() => {
    const ref = wrapperRef.current;
    const wheelHandler = (e: WheelEvent) => {
      e.preventDefault();
      const delta = Math.sign(e.deltaY);
      if (delta < 0) {
        zoomIn();
      } else {
        zoomOut();
      }
    };

    if (chartLoaded && ref) {
      ref.addEventListener("wheel", wheelHandler, { passive: false });
    }

    return () => {
      if (ref) {
        ref.removeEventListener("wheel", wheelHandler);
      }
    };
  }, [chartLoaded, zoomIn, zoomOut]);

  return {
    wrapperRef,
    clipPathRefs,
    gridRef,
    xPadding,
    mousePositionToGrid,
    onChartMouseDown,
    onChartMouseUp,
    setWrapperRef,
    onChartMouseMove,
    zoomOut,
    zoomIn,
  };
};
