export interface MenuItemData {
  uid?: string;
  label?: string;
  leftIcon?: React.ReactNode;
  rightIcon?: React.ReactNode;
  callback?: () => void;
  items?: MenuItemData[];
}
