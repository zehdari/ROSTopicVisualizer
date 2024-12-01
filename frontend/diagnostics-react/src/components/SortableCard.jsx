import React from "react";
import { useSortable } from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
export function SortableCard({ children, id }) {
  const { attributes, listeners, setNodeRef, transform, transition } =
    useSortable({ id });
  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
  };
  // Clone the child element and pass dragging props
  const childWithDraggableHeader = React.Children.map(children, (child) => {
    return React.cloneElement(child, {
      headerProps: {
        style: {
          cursor: "grab",
          touchAction: "none",
        },
        className: "graph-header",
        ...attributes,
        ...listeners,
      },
    });
  });
  return (
    <div ref={setNodeRef} style={style}>
      {childWithDraggableHeader}
    </div>
  );
}
export default SortableCard;
