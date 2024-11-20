if (typeof Plotly === "undefined") {
  console.error(
    "Plotly library not loaded. Ensure the CDN or local file is accessible."
  );
} else {
  console.log("Plotly library loaded successfully.");
}

const socket = io.connect("http://localhost:5000");
const charts = {}; // Store Plotly chart data and layout by topic
const timeWindow = 30 * 1000; // 30 seconds in milliseconds

socket.on("ros2_data", (msg) => {
  const topic = msg.topic;
  const data = msg.data;

  if (!charts[topic]) {
    createPlotForTopic(topic, data);
  }

  const timestamp = new Date(); // Create a JavaScript Date object for the current time
  updatePlotData(charts[topic], data, timestamp);
});

function createPlotForTopic(topic, data) {
  const container = document.createElement("div");
  container.className = "chart-container";

  const title = document.createElement("div");
  title.className = "chart-title";
  title.textContent = topic;
  container.appendChild(title);

  const plotDiv = document.createElement("div");
  plotDiv.id = `plot-${topic}`;
  plotDiv.style.width = "100%";
  plotDiv.style.height = "100%";
  container.appendChild(plotDiv);

  document.getElementById("charts-container").appendChild(container);

  const traces = generateTraces(data);
  charts[topic] = {
    divId: plotDiv.id,
    data: traces,
    layout: {
      xaxis: {
        title: "Time",
        showgrid: true,
        type: "date", // Interpret timestamps as dates
        tickformat: "%H:%M:%S", // Show time in hh:mm:ss format
        range: [new Date(Date.now() - timeWindow), new Date()],
      },
      yaxis: { title: "Value", showgrid: true },
      margin: { t: 10, l: 40, r: 10, b: 50 }, // Adjusted bottom margin
      height: 300,
    },
  };

  Plotly.newPlot(plotDiv.id, charts[topic].data, charts[topic].layout);
}

function generateTraces(data) {
  const traces = [];
  parseData(data, "", traces);
  return traces;
}

function parseData(data, prefix, traces) {
  for (const [key, value] of Object.entries(data)) {
    if (typeof value === "object" && value !== null) {
      parseData(value, `${prefix}${key}.`, traces);
    } else {
      traces.push({
        x: [],
        y: [],
        mode: "lines",
        name: `${prefix}${key}`,
      });
    }
  }
}

function updatePlotData(chart, data, timestamp) {
  const flatData = {};
  flattenData(data, "", flatData);

  chart.data.forEach((trace) => {
    const key = trace.name;

    trace.x.push(timestamp); // Push the formatted time for x-axis
    trace.y.push(flatData[key] || 0); // Push the data value for y-axis

    // Remove points outside the time window
    const minTime = new Date(timestamp.getTime() - timeWindow);
    while (trace.x.length > 0 && trace.x[0] < minTime) {
      trace.x.shift();
      trace.y.shift();
    }
  });

  // Synchronize the x-axis range for all graphs
  const currentTime = timestamp;
  chart.layout.xaxis.range = [
    new Date(currentTime.getTime() - timeWindow),
    currentTime,
  ];

  Plotly.update(chart.divId, chart.data, chart.layout);
}

function flattenData(data, prefix, result) {
  for (const [key, value] of Object.entries(data)) {
    if (typeof value === "object" && value !== null) {
      flattenData(value, `${prefix}${key}.`, result);
    } else {
      // Convert booleans to 1 or 0
      result[`${prefix}${key}`] =
        typeof value === "boolean" ? (value ? 1 : 0) : value;
    }
  }
}
