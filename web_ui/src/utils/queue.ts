export default class Queue<Type> {
  private maxSize: number;
  private data: Type[];
  constructor(maxSize = 1) {
    this.maxSize = maxSize;
    this.data = [];
  }
  enque(x: Type) {
    this.data.push(x);
    if (this.data.length > this.maxSize) {
      this.data.shift();
    }
  }
  deque() {
    return this.data.shift();
  }
  empty() {
    return this.data.length === 0;
  }
  size() {
    return this.data.length;
  }
}
