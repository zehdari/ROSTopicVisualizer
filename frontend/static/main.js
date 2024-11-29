const socket = io.connect("http://localhost:5000");
const charts = {}; // Store Plotly chart data and layout by topic
const timeWindow = 30 * 1000; // Time window for charts

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

  // Add a div for topic name and rate
  const header = document.createElement("div");
  header.style.display = "flex";
  header.style.justifyContent = "space-between";
  header.style.width = "100%";

  const title = document.createElement("div");
  title.className = "chart-title";
  title.textContent = topic;

  const rateDiv = document.createElement("div");
  rateDiv.id = `rate-${topic}`;
  rateDiv.style.fontSize = "12px";
  rateDiv.style.color = "#888";
  rateDiv.style.marginRight = "10px";
  rateDiv.textContent = "Hz: Calculating...";

  header.appendChild(title);
  header.appendChild(rateDiv);
  container.appendChild(header);

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
        type: "date",
        tickformat: "%H:%M:%S",
        range: [new Date(Date.now() - timeWindow), new Date()],
      },
      yaxis: { title: "Value", showgrid: true },
      margin: { t: 10, l: 40, r: 10, b: 50 },
      height: 300,
    },
    rateDivId: rateDiv.id,
    timestamps: [],
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

setInterval(() => {
  const currentTime = new Date();

  Object.values(charts).forEach((chart) => {
    const newRange = [
      new Date(currentTime.getTime() - timeWindow),
      currentTime,
    ];
    chart.layout.xaxis.range = newRange;

    // Check if the topic has stopped publishing
    if (chart.lastReceived && currentTime - chart.lastReceived > 1000) {
      document.getElementById(chart.rateDivId).textContent = `Hz: 0.00`;
    }

    // Update the plot with the new range (no data changes)
    Plotly.update(chart.divId, chart.data, chart.layout);
  });
}, 100); // Update every 100ms

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

  // Update the last received timestamp for this chart
  chart.lastReceived = timestamp;

  // Synchronize the x-axis range for all graphs
  const currentTime = timestamp;
  chart.layout.xaxis.range = [
    new Date(currentTime.getTime() - timeWindow),
    currentTime,
  ];

  // Calculate and update the Hz
  chart.timestamps.push(timestamp);
  const hzWindow = 1500; // 1.5 seconds window for stability
  const minHzTime = new Date(timestamp.getTime() - hzWindow);
  chart.timestamps = chart.timestamps.filter((t) => t >= minHzTime);

  if (chart.timestamps.length > 1) {
    const timeDifferences = chart.timestamps
      .map((t, i, arr) => {
        if (i === 0) return 0; // No difference for the first timestamp
        return (t - arr[i - 1]) / 1000; // Difference in seconds
      })
      .filter((diff) => diff > 0); // Filter valid intervals

    const avgHz =
      timeDifferences.length > 0
        ? 1 / (timeDifferences.reduce((a, b) => a + b) / timeDifferences.length)
        : 0;
    const roundedHz = avgHz.toFixed(2);

    document.getElementById(chart.rateDivId).textContent = `Hz: ${roundedHz}`;
  } else {
    // If only one timestamp, display a default minimum Hz
    document.getElementById(chart.rateDivId).textContent = `Hz: 1.00`;
  }

  Plotly.update(chart.divId, chart.data, chart.layout);
}

window.addEventListener("resize", () => {
  Object.values(charts).forEach((chart) => {
    // Get the chart container element
    const plotDiv = document.getElementById(chart.divId);

    // Update the layout dimensions based on the container size
    const newWidth = plotDiv.offsetWidth;
    const newHeight = 300; // Adjust height as needed

    Plotly.relayout(chart.divId, {
      width: newWidth,
      height: newHeight,
    });
  });
});

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
